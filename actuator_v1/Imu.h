#pragma once
// ===========================================================================
//  Imu.h  --  absolute angle from gravity, for a tilted rotation axis
// ===========================================================================
//
//  WHY AN ACCELEROMETER AND NOT A COMPASS
//
//  The yaw axis on this mount is tilted (a TVRO polar mount points its axis
//  at the celestial pole, so at Davis it sits ~51 deg off vertical). Rotating
//  about a tilted axis TILTS THE DISH, and tilt is something gravity measures
//  directly.
//
//  Rotate a body by theta about an axis tilted beta from vertical. In the body
//  frame the component of gravity along the axis is constant; the component
//  PERPENDICULAR to the axis has magnitude sin(beta) and sweeps around 1:1
//  with theta. So the direction of that perpendicular component IS the yaw
//  angle, with unity gain across the whole arc -- no flat spot, and no
//  ambiguity, because we use the full 3-vector rather than a scalar tilt.
//
//  Error propagates as  sigma_theta = sigma_tilt / sin(beta). At beta = 51.5
//  that is a 1.28x penalty, so 0.1 deg of tilt noise buys 0.13 deg of yaw.
//  A magnetometer on this mount would manage 1-2 deg at best: an 8 ft steel
//  reflector is an enormous soft-iron distorter, and while it rotates WITH
//  the sensor (so an ellipsoid fit removes it) the pier does not -- it stays
//  fixed in the earth frame while the sensor sweeps through it, leaving a
//  heading-dependent residual that no static calibration can reach.
//
//  For an 8 ft dish at 1694 MHz the beam is 5.08 deg wide and 2.54 deg off
//  boresight already costs 3 dB. Gravity clears that by 20x. Magnetics do not.
//
//  WHAT THIS FILE DELIBERATELY DOES NOT DO
//
//  It does not compute yaw. It reports an averaged, quality-gated gravity
//  unit vector and nothing more. The axis fit and the counts<->degrees model
//  live in host/axis_fit.py, for the same reason the satellite math does:
//  the host has floats, a config file, and tests. Keeping the trig off the
//  328P is what makes this affordable in flash.
//
//  READ ONLY WHEN STOPPED. An accelerometer measures gravity plus whatever
//  else is accelerating it, and an 8 ft dish is a large sail. Every read here
//  is averaged and then gated on sample variance: if the mount is ringing in
//  the wind, the batch is rejected rather than believed.
//
//  The register maps below are from the datasheets and have NOT been checked
//  against silicon -- nothing in this file has been run. Verify WHO_AM_I
//  answers before trusting anything downstream of it.

#include <Arduino.h>
#include "Config.h"

#if USE_IMU

#include <Wire.h>

// --- register maps ---------------------------------------------------------

#if IMU_SENSOR == IMU_MPU9150
  // MPU-9150 = MPU-6050 core + AK8975 magnetometer. We use only the accel.
  static const uint8_t IMU_REG_WHOAMI   = 0x75;
  static const uint8_t IMU_WHOAMI_VALUE = 0x68;
  static const uint8_t MPU_PWR_MGMT_1   = 0x6B;
  static const uint8_t MPU_CONFIG       = 0x1A;
  static const uint8_t MPU_ACCEL_CONFIG = 0x1C;
  static const uint8_t MPU_ACCEL_XOUT_H = 0x3B;
  static const uint8_t MPU_TEMP_OUT_H   = 0x41;
  // +/-2 g gives the finest scale the part has: 16384 LSB/g, 0.061 mg/LSB.
  static const float   IMU_LSB_PER_G    = 16384.0f;

#elif IMU_SENSOR == IMU_FXOS8700
  static const uint8_t IMU_REG_WHOAMI   = 0x0D;
  static const uint8_t IMU_WHOAMI_VALUE = 0xC7;
  static const uint8_t FXOS_XYZ_DATA_CFG = 0x0E;
  static const uint8_t FXOS_CTRL_REG1    = 0x2A;
  static const uint8_t FXOS_CTRL_REG2    = 0x2B;
  static const uint8_t FXOS_OUT_X_MSB    = 0x01;
  static const uint8_t FXOS_TEMP         = 0x51;
  static const uint8_t FXOS_M_CTRL_REG1  = 0x5B;
  // 14-bit left-justified in 16 bits. After the >>2 shift, +/-2 g is
  // 4096 LSB/g (0.244 mg/LSB) -- 4x coarser than the MPU, but the part is
  // roughly 3x quieter by noise density, and noise is what actually limits us.
  static const float   IMU_LSB_PER_G     = 4096.0f;

#else
  #error "Set IMU_SENSOR to IMU_MPU9150 or IMU_FXOS8700"
#endif

// --- state -----------------------------------------------------------------

struct ImuSample {
  float x, y, z;      // unit vector along gravity, sensor frame
  float magnitude_g;  // length before normalising; should be ~1.000
  float spread_g;     // RMS deviation of |a| across the batch -- the gate
  float temp_c;
  uint8_t n;          // samples actually averaged
  bool ok;
};

bool  g_imuOk = false;    // false if WHO_AM_I never answered

// --- I2C plumbing ----------------------------------------------------------

static bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool imuReadBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_I2C_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// --- bring-up --------------------------------------------------------------

void imuBegin() {
  Wire.begin();

  uint8_t who = 0;
  if (!imuReadBytes(IMU_REG_WHOAMI, &who, 1) || who != IMU_WHOAMI_VALUE) {
    g_imuOk = false;
    return;
  }

#if IMU_SENSOR == IMU_MPU9150
  // Out of sleep. 0x00 selects the internal oscillator, which is fine: we
  // never use the gyro, so there is no PLL worth locking to.
  imuWrite(MPU_PWR_MGMT_1, 0x00);
  delay(50);
  // DLPF_CFG = 6 is the narrowest on-chip filter (~5 Hz accel bandwidth).
  // We are measuring a stationary dish, so every Hz of bandwidth above that
  // is wind and gearbox ring that we would only have to average back out.
  imuWrite(MPU_CONFIG, 0x06);
  imuWrite(MPU_ACCEL_CONFIG, 0x00);   // AFS_SEL = 0 -> +/-2 g

#elif IMU_SENSOR == IMU_FXOS8700
  // CTRL_REG1 must be in standby before any config register will take.
  imuWrite(FXOS_CTRL_REG1, 0x00);
  delay(10);
  imuWrite(FXOS_M_CTRL_REG1, 0x00);   // magnetometer off -- accel only
  imuWrite(FXOS_XYZ_DATA_CFG, 0x00);  // +/-2 g
  imuWrite(FXOS_CTRL_REG2, 0x02);     // high-resolution oversampling
  // ODR 50 Hz (dr = 0b100 in bits 5:3), then active.
  imuWrite(FXOS_CTRL_REG1, (0x04 << 3) | 0x01);
  delay(100);
#endif

  g_imuOk = true;
}

// --- one raw triple --------------------------------------------------------

static bool imuReadRaw(float* ax, float* ay, float* az) {
  uint8_t b[6];

#if IMU_SENSOR == IMU_MPU9150
  if (!imuReadBytes(MPU_ACCEL_XOUT_H, b, 6)) return false;
  const int16_t rx = (int16_t)((uint16_t)b[0] << 8 | b[1]);
  const int16_t ry = (int16_t)((uint16_t)b[2] << 8 | b[3]);
  const int16_t rz = (int16_t)((uint16_t)b[4] << 8 | b[5]);

#elif IMU_SENSOR == IMU_FXOS8700
  if (!imuReadBytes(FXOS_OUT_X_MSB, b, 6)) return false;
  // 14-bit data sits left-justified in the 16-bit pair, so the arithmetic
  // shift has to happen after the cast to signed or the sign bit is lost.
  const int16_t rx = (int16_t)((uint16_t)b[0] << 8 | b[1]) >> 2;
  const int16_t ry = (int16_t)((uint16_t)b[2] << 8 | b[3]) >> 2;
  const int16_t rz = (int16_t)((uint16_t)b[4] << 8 | b[5]) >> 2;
#endif

  *ax = (float)rx / IMU_LSB_PER_G;
  *ay = (float)ry / IMU_LSB_PER_G;
  *az = (float)rz / IMU_LSB_PER_G;
  return true;
}

static float imuTemperature() {
#if IMU_SENSOR == IMU_MPU9150
  uint8_t t[2];
  if (!imuReadBytes(MPU_TEMP_OUT_H, t, 2)) return -273.0f;
  const int16_t raw = (int16_t)((uint16_t)t[0] << 8 | t[1]);
  return (float)raw / 340.0f + 36.53f;
#elif IMU_SENSOR == IMU_FXOS8700
  uint8_t t;
  if (!imuReadBytes(FXOS_TEMP, &t, 1)) return -273.0f;
  return (float)(int8_t)t * 0.96f;
#endif
}

// --- the averaged, gated read ----------------------------------------------
//
//  Temperature comes back with every sample because accelerometer zero-g
//  offset moves with it, and that -- not resolution, not noise -- is the
//  dominant error outdoors. One mg of offset is 0.057 deg of tilt, so a
//  30 C day/night swing on a part drifting ~1 mg/C is over a degree. Log the
//  temperature with every calibration point and the drift is at least
//  measurable; ignore it and it silently eats the margin.

ImuSample imuRead(uint8_t samples, uint16_t gapMs) {
  ImuSample s;
  s.x = s.y = s.z = 0.0f;
  s.magnitude_g = s.spread_g = 0.0f;
  s.temp_c = -273.0f;
  s.n = 0;
  s.ok = false;

  if (!g_imuOk) return s;

  float sx = 0, sy = 0, sz = 0;
  float sumMag = 0, sumMagSq = 0;

  for (uint8_t i = 0; i < samples; i++) {
    float ax, ay, az;
    if (!imuReadRaw(&ax, &ay, &az)) return s;

    const float mag = sqrt(ax * ax + ay * ay + az * az);
    sx += ax; sy += ay; sz += az;
    sumMag += mag;
    sumMagSq += mag * mag;
    s.n++;

    if (gapMs) delay(gapMs);
  }

  const float inv = 1.0f / (float)s.n;
  sx *= inv; sy *= inv; sz *= inv;

  const float meanMag = sumMag * inv;
  // Population variance of |a|. A stationary dish sits near zero; a dish
  // being pushed around by wind does not, and that is the whole test.
  float var = sumMagSq * inv - meanMag * meanMag;
  if (var < 0.0f) var = 0.0f;          // rounding, not physics
  s.spread_g = sqrt(var);

  const float norm = sqrt(sx * sx + sy * sy + sz * sz);
  if (norm < 0.1f) return s;           // nothing sane came back

  s.x = sx / norm;
  s.y = sy / norm;
  s.z = sz / norm;
  s.magnitude_g = norm;
  s.temp_c = imuTemperature();

  // Two independent ways to be wrong, both worth catching:
  //   - the batch was noisy (wind, someone leaning on the dish)
  //   - the mean is not 1 g, which means bias, wrong scale, or a bad part
  s.ok = (s.spread_g <= IMU_MAX_SPREAD_G) &&
         (fabs(norm - 1.0f) <= IMU_MAX_MAG_ERROR_G);
  return s;
}

#else   // !USE_IMU

struct ImuSample { float x, y, z, magnitude_g, spread_g, temp_c; uint8_t n; bool ok; };
bool g_imuOk = false;
inline void imuBegin() {}
inline ImuSample imuRead(uint8_t, uint16_t) { ImuSample s; s.ok = false; s.n = 0; return s; }

#endif  // USE_IMU
