#include <Arduino.h>
#include <Wire.h>
#include <math.h>



constexpr int IMU_SDA_PIN = 1;
constexpr int IMU_SCL_PIN = 2;
constexpr uint8_t IMU_ADDRESS = 0x6A;
constexpr uint8_t EXPECTED_WHO_AM_I = 0x6C;

constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G  = 0x11;
constexpr uint8_t REG_CTRL3_C  = 0x12;
constexpr uint8_t REG_OUTX_L_G = 0x22;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t firstRegister, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(firstRegister);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(IMU_ADDRESS, length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

int16_t makeInt16(uint8_t lowByte, uint8_t highByte) {
  const uint16_t value = (static_cast<uint16_t>(highByte) << 8) | lowByte;
  return static_cast<int16_t>(value);
}

bool initialiseIMU() {
  uint8_t whoAmI = 0;

  if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1)) {
    Serial.println("ERROR: no response from IMU at I2C address 0x6A");
    return false;
  }

  Serial.printf("IMU WHO_AM_I = 0x%02X\n", whoAmI);
  if (whoAmI != EXPECTED_WHO_AM_I) {
    Serial.println("ERROR: unexpected IMU ID (expected LSM6DSO: 0x6C)");
    return false;
  }


  if (!writeRegister(REG_CTRL3_C, 0x44)) return false;

  if (!writeRegister(REG_CTRL1_XL, 0x40)) return false;

  if (!writeRegister(REG_CTRL2_G, 0x40)) return false;

  delay(100);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("CANSAT IMU MVP");
  Serial.println("ESP32-S3 + LSM6DSO");


  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);

  Serial.printf("I2C: SDA=GPIO%d, SCL=GPIO%d, address=0x%02X\n",
                IMU_SDA_PIN, IMU_SCL_PIN, IMU_ADDRESS);

  if (!initialiseIMU()) {
    Serial.println("IMU init failed. Check 3V3, GND, SDA, SCL and CS.");
    while (true) delay(1000);
  }

  Serial.println("IMU initialised.");
  Serial.println("time_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg");
}

void loop() {

  uint8_t data[12];
  if (!readRegisters(REG_OUTX_L_G, data, sizeof(data))) {
    Serial.printf("%lu,ERROR,I2C_READ\n", static_cast<unsigned long>(millis()));
    delay(20);
    return;
  }

  const int16_t gxRaw = makeInt16(data[0],  data[1]);
  const int16_t gyRaw = makeInt16(data[2],  data[3]);
  const int16_t gzRaw = makeInt16(data[4],  data[5]);
  const int16_t axRaw = makeInt16(data[6],  data[7]);
  const int16_t ayRaw = makeInt16(data[8],  data[9]);
  const int16_t azRaw = makeInt16(data[10], data[11]);


  const float ax = axRaw * (2.0f / 32768.0f); // g
  const float ay = ayRaw * (2.0f / 32768.0f); // g
  const float az = azRaw * (2.0f / 32768.0f); // g
  const float gx = gxRaw * 0.00875f;          // degrees/second
  const float gy = gyRaw * 0.00875f;
  const float gz = gzRaw * 0.00875f;


  constexpr float RAD_TO_DEG = 57.2957795f;
  const float roll = atan2f(ay, az) * RAD_TO_DEG;
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                static_cast<unsigned long>(millis()),
                ax, ay, az, gx, gy, gz, roll, pitch);

  delay(20); .
}
