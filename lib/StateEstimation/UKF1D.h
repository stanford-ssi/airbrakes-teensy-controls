#pragma once

// 1D Unscented Kalman Filter
// State: [altitude (m), velocity (m/s), acceleration (m/s²)]
// Process model: constant acceleration
// Measurements: accelerometer (scalar) and barometer (scalar), updated independently

class UKF1D {
public:
    static constexpr int N = 3;          // State dimension
    static constexpr int NUM_SIGMA = 2 * N + 1;  // 7 sigma points

    UKF1D();

    // Initialize state and covariance
    void init(float alt0, float vel0, float accel0);

    // Predict step: propagate state forward by dt seconds
    void predict(float dt);

    // Measurement update with accelerometer (measures acceleration)
    void updateAccel(float meas_accel, float R);

    // Measurement update with barometer (measures altitude)
    void updateBaro(float meas_alt, float R);

    // Set process noise standard deviations
    void setProcessNoise(float q_alt, float q_vel, float q_accel);

    // Getters
    float altitude() const { return x_[0]; }
    float velocity() const { return x_[1]; }
    float acceleration() const { return x_[2]; }

    bool isInitialized() const { return initialized_; }

private:
    // State vector [alt, vel, accel]
    float x_[N];

    // Covariance matrix (symmetric 3x3, stored as flat array)
    float P_[N * N];

    // Process noise covariance
    float Q_[N * N];

    bool initialized_;

    // Merwe's scaled sigma point parameters
    static constexpr float ALPHA = 1e-3f;
    static constexpr float BETA = 2.0f;
    static constexpr float KAPPA = 0.0f;
    static constexpr float LAMBDA = ALPHA * ALPHA * (N + KAPPA) - N;

    // Weights
    float Wm_[NUM_SIGMA];  // Mean weights
    float Wc_[NUM_SIGMA];  // Covariance weights

    // Sigma points storage
    float sigmas_[NUM_SIGMA][N];

    void computeWeights();
    void generateSigmaPoints();

    // Cholesky decomposition of P into L (lower triangular)
    // Returns false if P is not positive definite
    bool cholesky(const float *P, float *L);

    // Helper: 3x3 matrix operations (all inline, stack-allocated)
    void matAdd(const float *A, const float *B, float *C);
    void matSub(const float *A, const float *B, float *C);
    float matGet(const float *M, int r, int c) const { return M[r * N + c]; }
    void matSet(float *M, int r, int c, float v) { M[r * N + c] = v; }
};
