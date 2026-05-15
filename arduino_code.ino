#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// rotation matrix: body -> world
float R[3][3] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};

unsigned long prevTime = 0;

// correction gain for accelerometer feedback
const float Kacc = 0.15f;

// helpers 
float dot3(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

float norm3(const float v[3]) {
  return sqrt(dot3(v, v));
}

bool normalize3(float v[3]) {
  float n = norm3(v);
  if (n < 1e-6f) return false;
  v[0] /= n;
  v[1] /= n;
  v[2] /= n;
  return true;
}

void matMul33(const float A[3][3], const float B[3][3], float C[3][3]) {
  float tmp[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      tmp[i][j] = 0.0f;
      for (int k = 0; k < 3; k++) {
        tmp[i][j] += A[i][k] * B[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C[i][j] = tmp[i][j];
    }
  }
}

// Gram-Schmidt orthonormalization
void orthonormalizeR() {
  float x[3] = {R[0][0], R[1][0], R[2][0]};
  float y[3] = {R[0][1], R[1][1], R[2][1]};
  float z[3];

  normalize3(x);

  // y = y - proj_x(y)
  float proj = dot3(x, y);
  y[0] -= proj * x[0];
  y[1] -= proj * x[1];
  y[2] -= proj * x[2];
  normalize3(y);

  // z = x × y
  cross3(x, y, z);
  normalize3(z);

  // write back as columns
  R[0][0] = x[0]; R[1][0] = x[1]; R[2][0] = x[2];
  R[0][1] = y[0]; R[1][1] = y[1]; R[2][1] = y[2];
  R[0][2] = z[0]; R[1][2] = z[1]; R[2][2] = z[2];
}

// convert rotation matrix to roll/pitch/yaw in degrees
void matrixToEuler(float &roll, float &pitch, float &yaw) {
  // body -> world, ZYX convention
  pitch = -asin(R[2][0]);
  roll  = atan2(R[2][1], R[2][2]);
  yaw   = atan2(R[1][0], R[0][0]);

  roll  *= 180.0f / PI;
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
}

void setup() {
  Serial.begin(19200);
  while (!Serial) {
    delay(10);
  }

  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(1000);
  prevTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0f;
  prevTime = currentTime;

  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  // measured acceleration vector in body frame
  float a_meas[3] = {
    a.acceleration.x,
    a.acceleration.y,
    a.acceleration.z
  };

  bool accelOK = normalize3(a_meas);

  // gravity in world frame is [0,0,-1]
  float g_pred[3] = {
    -R[0][2],
    -R[1][2],
    -R[2][2]
  };

  // gyro vector in rad/s
  float omega[3] = {
    g.gyro.x,
    g.gyro.y,
    g.gyro.z
  };

  // accelerometer correction using cross product
  if (accelOK) {
    float error[3];
    cross3(a_meas, g_pred, error);

    omega[0] += Kacc * error[0];
    omega[1] += Kacc * error[1];
    omega[2] += Kacc * error[2];
  }

  // skew-symmetric matrix Omega from corrected omega
  float Omega[3][3] = {
    {0.0f,      -omega[2],  omega[1]},
    {omega[2],   0.0f,     -omega[0]},
    {-omega[1],  omega[0],  0.0f}
  };

  // first-order integration: R_{k+1} = R_k (I + Omega dt)
  float dR[3][3] = {
    {1.0f + Omega[0][0]*dt, Omega[0][1]*dt,      Omega[0][2]*dt},
    {Omega[1][0]*dt,        1.0f + Omega[1][1]*dt, Omega[1][2]*dt},
    {Omega[2][0]*dt,        Omega[2][1]*dt,      1.0f + Omega[2][2]*dt}
  };

  matMul33(R, dR, R);
  orthonormalizeR();

  float roll, pitch, yaw;
  matrixToEuler(roll, pitch, yaw);

  // print for Processing
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

  delay(10);
}