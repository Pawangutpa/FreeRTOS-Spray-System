#include <Arduino.h>
#include "drivers/ec200u_driver.h"
#include "config/ec200u_config.h"
#include "config/mqtt_config.h"
#include "common/kalman_filter.h"
#include "rtos/rtos_queues.h"

static HardwareSerial Modem(EC200U_UART_PORT);

#define CMD_TIMEOUT_MS 5000
#define CMD_RETRY_LIMIT 3

// ================= RX BUFFER =================

static char rxLine[512];
static int rxIndex = 0;

// ================= STATE MACHINE =================

enum ECState {
    ST_BOOT,

    ST_SEND_ATE0, ST_WAIT_ATE0,
    ST_SEND_CMEE, ST_WAIT_CMEE,
    ST_SEND_CPIN, ST_WAIT_SIM,

    ST_SEND_CREG, ST_WAIT_NETWORK,
    ST_SEND_CGATT, ST_WAIT_ATTACH,

    ST_DEACT_PDP, ST_WAIT_DEACT,
    ST_SEND_CGDCONT,
    ST_SEND_QICSGP,
    ST_SEND_QIACT, ST_WAIT_PDP,

    ST_START_GPS,
    ST_CONFIG_MQTT,

    ST_OPEN_MQTT, ST_WAIT_MQTT_OPEN,
    ST_CONNECT_MQTT, ST_WAIT_MQTT_CONN,

    ST_READY
};

static ECState state = ST_BOOT;
static unsigned long stateStart = 0;
static int retryCount = 0;
static bool mqttReady = false;

static KalmanFilter kLat, kLon;
static bool kalmanInit = false;

// ================= UTIL =================

static void changeState(ECState newState)
{
    state = newState;
    stateStart = millis();
    retryCount = 0;
}

static void atSend(const char *cmd)
{
    Serial.print("TX -> ");
    Serial.println(cmd);
    Modem.print(cmd);
    Modem.print("\r\n");
    stateStart = millis();
}

static bool timeout()
{
    return millis() - stateStart > CMD_TIMEOUT_MS;
}

static void trimCR(char* s)
{
    int len = strlen(s);
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n'))
    {
        s[len-1] = 0;
        len--;
    }
}

static double nmeaToDecimal(double raw, char dir)
{
    int deg = int(raw / 100);
    double min = raw - deg * 100;
    double dec = deg + min / 60.0;
    if (dir == 'S' || dir == 'W')
        dec *= -1;
    return dec;
}

// ================= RESPONSE HANDLER =================

static void handleLine(char *line)
{
    trimCR(line);

    if (strlen(line) == 0)
        return;

    Serial.print("RX <- ");
    Serial.println(line);

    switch(state)
    {
        case ST_WAIT_ATE0:
            if (strstr(line, "OK"))
                changeState(ST_SEND_CMEE);
            break;

        case ST_WAIT_CMEE:
            if (strstr(line, "OK"))
                changeState(ST_SEND_CPIN);
            break;

        case ST_WAIT_SIM:
            if (strstr(line, "READY"))
                changeState(ST_SEND_CREG);
            break;

        case ST_WAIT_NETWORK:
            if (strstr(line, "+CREG: 0,1") ||
                strstr(line, "+CREG: 0,5"))
                changeState(ST_SEND_CGATT);
            break;

        case ST_WAIT_ATTACH:
            if (strstr(line, "+CGATT: 1"))
                changeState(ST_DEACT_PDP);
            break;

        case ST_WAIT_DEACT:
            if (strstr(line, "OK") || strstr(line, "ERROR"))
                changeState(ST_SEND_CGDCONT);
            break;

        case ST_WAIT_PDP:
            if (strstr(line, "OK"))
                changeState(ST_START_GPS);
            break;

        case ST_WAIT_MQTT_OPEN:
            if (strstr(line, "+QMTOPEN: 0,0"))
                changeState(ST_CONNECT_MQTT);
            break;

        case ST_WAIT_MQTT_CONN:
            if (strstr(line, "+QMTCONN: 0,0,0"))
            {
                mqttReady = true;
                Serial.println("MQTT CONNECTED");
                changeState(ST_READY);
            }
            break;

        case ST_READY:

            if (strstr(line, "+QMTPUBEX: 0,0,0"))
                Serial.println("Publish OK");

            if (strstr(line, "+CSQ:"))
            {
                int n;
                sscanf(line, "+CSQ: %d,", &n);
                int rssi = (n == 99) ? 0 : (-113 + 2*n);

                GpsData g = {};
                g.rssi = rssi;
                xQueueSend(qGpsData, &g, 0);
            }

            if (strstr(line, "+QGPSLOC:"))
            {
                char t[16], latStr[20], lonStr[20];
                int fix;

                sscanf(line,
                    "+QGPSLOC: %[^,],%[^,],%[^,],%*[^,],%*[^,],%d",
                    t, latStr, lonStr, &fix);

                char latDir = latStr[strlen(latStr)-1];
                char lonDir = lonStr[strlen(lonStr)-1];
                latStr[strlen(latStr)-1] = 0;
                lonStr[strlen(lonStr)-1] = 0;

                double lat = nmeaToDecimal(atof(latStr), latDir);
                double lon = nmeaToDecimal(atof(lonStr), lonDir);

                if (!kalmanInit)
                {
                    kLat.X = lat;
                    kLon.X = lon;
                    kalmanInit = true;
                }

                GpsData g = {};
                g.lat = kLat.update(lat);
                g.lon = kLon.update(lon);
                g.fix = fix;

                xQueueSend(qGpsData, &g, 0);
            }

            break;

        default:
            break;
    }
}

// ================= PUBLIC =================

void EC200U_init()
{
    Modem.begin(EC200U_BAUD, SERIAL_8N1,
                EC200U_RX, EC200U_TX);
    changeState(ST_BOOT);
}

bool EC200U_isReady()
{
    return mqttReady;
}

void EC200U_publish(const char *topic, const char *payload)
{
    if (!mqttReady) return;

    char cmd[256];
    sprintf(cmd,
        "AT+QMTPUBEX=0,0,0,0,\"%s\",%d",
        topic,
        strlen(payload));

    atSend(cmd);
    delay(20);
    Modem.print(payload);
}

void EC200U_process()
{
    while (Modem.available())
{
    char c = Modem.read();

    if (c == '\r')
        continue;   // ignore CR

    if (c == '\n')
    {
        if (rxIndex == 0)
            continue;

        rxLine[rxIndex] = 0;
        handleLine(rxLine);
        rxIndex = 0;
    }
    else
    {
        if (rxIndex < sizeof(rxLine)-1)
            rxLine[rxIndex++] = c;
    }
}

    switch(state)
    {
        case ST_BOOT:
            if (millis() > EC_BOOT_DELAY_MS)
                changeState(ST_SEND_ATE0);
            break;

        case ST_SEND_ATE0:
            atSend("ATE0");
            changeState(ST_WAIT_ATE0);
            break;

        case ST_SEND_CMEE:
            atSend("AT+CMEE=2");
            changeState(ST_WAIT_CMEE);
            break;

        case ST_SEND_CPIN:
            atSend("AT+CPIN?");
            changeState(ST_WAIT_SIM);
            break;

        case ST_SEND_CREG:
            atSend("AT+CREG?");
            changeState(ST_WAIT_NETWORK);
            break;

        case ST_SEND_CGATT:
            atSend("AT+CGATT?");
            changeState(ST_WAIT_ATTACH);
            break;

        case ST_DEACT_PDP:
            atSend("AT+QIDEACT=1");
            changeState(ST_WAIT_DEACT);
            break;

        case ST_SEND_CGDCONT:
            atSend("AT+CGDCONT=1,\"IP\",\"" EC200U_APN "\"");
            changeState(ST_SEND_QICSGP);
            break;

        case ST_SEND_QICSGP:
            atSend("AT+QICSGP=1,1,\"" EC200U_APN "\",\"\",\"\",1");
            changeState(ST_SEND_QIACT);
            break;

        case ST_SEND_QIACT:
            atSend("AT+QIACT=1");
            changeState(ST_WAIT_PDP);
            break;

        case ST_START_GPS:
            atSend("AT+QGPS=1");
            atSend("AT+QGPSCFG=\"nmeasrc\",1");
            changeState(ST_CONFIG_MQTT);
            break;

        case ST_CONFIG_MQTT:
            atSend("AT+QMTCFG=\"version\",0,4");
            atSend("AT+QMTCFG=\"clean\",0,1");
            atSend("AT+QMTCFG=\"keepalive\",0,30");
            atSend("AT+QMTCFG=\"retrans\",0,1");
            atSend("AT+QMTCLOSE=0");
            changeState(ST_OPEN_MQTT);
            break;

        case ST_OPEN_MQTT:
        {
            char cmd[200];
            sprintf(cmd,
                "AT+QMTOPEN=0,\"%s\",%d",
                MQTT_BROKER,
                MQTT_PORT);
            atSend(cmd);
            changeState(ST_WAIT_MQTT_OPEN);
            break;
        }

        case ST_CONNECT_MQTT:
        {
            char cmd[200];
            sprintf(cmd,
                "AT+QMTCONN=0,\"%s\"",
                MQTT_CLIENT_ID);
            atSend(cmd);
            changeState(ST_WAIT_MQTT_CONN);
            break;
        }

        case ST_READY:
{
    static unsigned long lastGpsPoll = 0;
    static unsigned long lastRssiPoll = 0;

    if (millis() - lastGpsPoll > 2000)
    {
        atSend("AT+QGPSLOC?");
        lastGpsPoll = millis();
    }

    if (millis() - lastRssiPoll > 5000)
    {
        atSend("AT+CSQ");
        lastRssiPoll = millis();
    }

    break;
}
        default:
            if (timeout())
            {
                if (++retryCount < CMD_RETRY_LIMIT)
                {
                    changeState(state);
                }
                else
                {
                    Serial.println("STATE FAILURE → RESET MODEM FLOW");
                    mqttReady = false;
                    changeState(ST_SEND_ATE0);
                }
            }
            break;
    }
}