#pragma once

class KalmanFilter {
public:
    float Q = 0.0009;
    float R = 0.0005;
    float P = 1;
    float X = 0;

    float update(float m) {
        P += Q;
        float K = P / (P + R);
        X += K * (m - X);
        P *= (1 - K);
        return X;
    }
};