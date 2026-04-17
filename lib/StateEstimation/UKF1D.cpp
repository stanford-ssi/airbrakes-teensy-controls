#include "UKF1D.h"
#include <math.h>
#include <string.h>

UKF1D::UKF1D() : initialized_(false) {
    memset(x_, 0, sizeof(x_));
    memset(P_, 0, sizeof(P_));
    memset(Q_, 0, sizeof(Q_));
    computeWeights();

    // Default process noise (std devs per step)
    setProcessNoise(0.05f, 0.2f, 0.3f);
}

void UKF1D::computeWeights() {
    float n_plus_lambda = N + LAMBDA;
    Wm_[0] = LAMBDA / n_plus_lambda;
    Wc_[0] = LAMBDA / n_plus_lambda + (1.0f - ALPHA * ALPHA + BETA);

    for (int i = 1; i < NUM_SIGMA; i++) {
        Wm_[i] = 1.0f / (2.0f * n_plus_lambda);
        Wc_[i] = 1.0f / (2.0f * n_plus_lambda);
    }
}

void UKF1D::init(float alt0, float vel0, float accel0) {
    x_[0] = alt0;
    x_[1] = vel0;
    x_[2] = accel0;

    // Initial covariance: moderate uncertainty
    memset(P_, 0, sizeof(P_));
    matSet(P_, 0, 0, 10.0f);   // altitude variance (m²)
    matSet(P_, 1, 1, 1.0f);    // velocity variance (m/s)²
    matSet(P_, 2, 2, 5.0f);    // acceleration variance (m/s²)²

    initialized_ = true;
}

void UKF1D::setProcessNoise(float q_alt, float q_vel, float q_accel) {
    memset(Q_, 0, sizeof(Q_));
    matSet(Q_, 0, 0, q_alt * q_alt);
    matSet(Q_, 1, 1, q_vel * q_vel);
    matSet(Q_, 2, 2, q_accel * q_accel);
}

bool UKF1D::cholesky(const float *P, float *L) {
    // 3x3 Cholesky decomposition: P = L * L^T
    memset(L, 0, N * N * sizeof(float));

    for (int i = 0; i < N; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0f;
            for (int k = 0; k < j; k++) {
                sum += L[i * N + k] * L[j * N + k];
            }
            if (i == j) {
                float val = P[i * N + i] - sum;
                if (val <= 0.0f) {
                    // Not positive definite, add jitter and retry
                    float Pfix[N * N];
                    memcpy(Pfix, P, sizeof(Pfix));
                    for (int d = 0; d < N; d++) {
                        Pfix[d * N + d] += 1e-6f;
                    }
                    if (val + 1e-6f <= 0.0f) return false;
                    return cholesky(Pfix, L);
                }
                L[i * N + j] = sqrtf(val);
            } else {
                L[i * N + j] = (P[i * N + j] - sum) / L[j * N + j];
            }
        }
    }
    return true;
}

void UKF1D::generateSigmaPoints() {
    float L[N * N];

    // Scale P by (N + LAMBDA) before Cholesky
    float scaled_P[N * N];
    float scale = N + LAMBDA;
    for (int i = 0; i < N * N; i++) {
        scaled_P[i] = P_[i] * scale;
    }

    cholesky(scaled_P, L);

    // Sigma point 0 = mean
    for (int j = 0; j < N; j++) {
        sigmas_[0][j] = x_[j];
    }

    // Sigma points 1..N: mean + column of L
    // Sigma points N+1..2N: mean - column of L
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            sigmas_[i + 1][j]     = x_[j] + L[j * N + i];
            sigmas_[i + 1 + N][j] = x_[j] - L[j * N + i];
        }
    }
}

void UKF1D::predict(float dt) {
    if (!initialized_) return;

    generateSigmaPoints();

    // Propagate each sigma point through constant-acceleration model
    // x' = [alt + vel*dt + 0.5*accel*dt², vel + accel*dt, accel]
    float propagated[NUM_SIGMA][N];
    for (int i = 0; i < NUM_SIGMA; i++) {
        float alt = sigmas_[i][0];
        float vel = sigmas_[i][1];
        float acc = sigmas_[i][2];

        propagated[i][0] = alt + vel * dt + 0.5f * acc * dt * dt;
        propagated[i][1] = vel + acc * dt;
        propagated[i][2] = acc; // Constant acceleration model
    }

    // Compute predicted mean
    float x_pred[N] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < NUM_SIGMA; i++) {
        for (int j = 0; j < N; j++) {
            x_pred[j] += Wm_[i] * propagated[i][j];
        }
    }

    // Compute predicted covariance
    float P_pred[N * N];
    memset(P_pred, 0, sizeof(P_pred));
    for (int i = 0; i < NUM_SIGMA; i++) {
        float diff[N];
        for (int j = 0; j < N; j++) {
            diff[j] = propagated[i][j] - x_pred[j];
        }
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) {
                P_pred[r * N + c] += Wc_[i] * diff[r] * diff[c];
            }
        }
    }

    // Add process noise
    for (int i = 0; i < N * N; i++) {
        P_pred[i] += Q_[i];
    }

    memcpy(x_, x_pred, sizeof(x_));
    memcpy(P_, P_pred, sizeof(P_));
}

void UKF1D::updateAccel(float meas_accel, float R) {
    if (!initialized_) return;

    generateSigmaPoints();

    // Measurement model: h(x) = x[2] (acceleration)
    float z_pred[NUM_SIGMA];
    for (int i = 0; i < NUM_SIGMA; i++) {
        z_pred[i] = sigmas_[i][2];
    }

    // Predicted measurement mean
    float z_mean = 0.0f;
    for (int i = 0; i < NUM_SIGMA; i++) {
        z_mean += Wm_[i] * z_pred[i];
    }

    // Innovation covariance S = sum(Wc * dz * dz) + R
    float S = R;
    for (int i = 0; i < NUM_SIGMA; i++) {
        float dz = z_pred[i] - z_mean;
        S += Wc_[i] * dz * dz;
    }

    // Cross covariance Pxz
    float Pxz[N] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < NUM_SIGMA; i++) {
        float dz = z_pred[i] - z_mean;
        for (int j = 0; j < N; j++) {
            Pxz[j] += Wc_[i] * (sigmas_[i][j] - x_[j]) * dz;
        }
    }

    // Kalman gain K = Pxz / S
    float K[N];
    for (int j = 0; j < N; j++) {
        K[j] = Pxz[j] / S;
    }

    // Update state
    float innovation = meas_accel - z_mean;
    for (int j = 0; j < N; j++) {
        x_[j] += K[j] * innovation;
    }

    // Update covariance: P = P - K * S * K^T
    for (int r = 0; r < N; r++) {
        for (int c = 0; c < N; c++) {
            P_[r * N + c] -= K[r] * S * K[c];
        }
    }
}

void UKF1D::updateBaro(float meas_alt, float R) {
    if (!initialized_) return;

    generateSigmaPoints();

    // Measurement model: h(x) = x[0] (altitude)
    float z_pred[NUM_SIGMA];
    for (int i = 0; i < NUM_SIGMA; i++) {
        z_pred[i] = sigmas_[i][0];
    }

    // Predicted measurement mean
    float z_mean = 0.0f;
    for (int i = 0; i < NUM_SIGMA; i++) {
        z_mean += Wm_[i] * z_pred[i];
    }

    // Innovation covariance
    float S = R;
    for (int i = 0; i < NUM_SIGMA; i++) {
        float dz = z_pred[i] - z_mean;
        S += Wc_[i] * dz * dz;
    }

    // Cross covariance
    float Pxz[N] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < NUM_SIGMA; i++) {
        float dz = z_pred[i] - z_mean;
        for (int j = 0; j < N; j++) {
            Pxz[j] += Wc_[i] * (sigmas_[i][j] - x_[j]) * dz;
        }
    }

    // Kalman gain
    float K[N];
    for (int j = 0; j < N; j++) {
        K[j] = Pxz[j] / S;
    }

    // Update state
    float innovation = meas_alt - z_mean;
    for (int j = 0; j < N; j++) {
        x_[j] += K[j] * innovation;
    }

    // Update covariance
    for (int r = 0; r < N; r++) {
        for (int c = 0; c < N; c++) {
            P_[r * N + c] -= K[r] * S * K[c];
        }
    }
}

void UKF1D::matAdd(const float *A, const float *B, float *C) {
    for (int i = 0; i < N * N; i++) C[i] = A[i] + B[i];
}

void UKF1D::matSub(const float *A, const float *B, float *C) {
    for (int i = 0; i < N * N; i++) C[i] = A[i] - B[i];
}
