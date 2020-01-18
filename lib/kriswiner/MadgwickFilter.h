// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(
    float q[],
    float beta,
    float deltat,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz);

__attribute__((optimize("O3"))) void MahonyQuaternionUpdate(
    float q[],
    float eInt[],
    float deltat,
    float Ki, float Kp,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz);
