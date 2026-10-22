
#include <Wire.h>

// ========================================
// MPU6050 CONFIGURATION
// ========================================

const uint8_t MPU_ADDR = 0x68;

const int SDA_PIN = 21;
const int SCL_PIN = 22;

// Sampling rate: 100 Hz
const uint32_t SAMPLE_PERIOD_US = 10000;

// Accelerometer: +/- 2g
// Sensitivity: 16384 LSB/g
const float ACCEL_SCALE = 9.80665f / 16384.0f;

// Gyroscope: +/- 1000 deg/s
// Sensitivity: 32.8 LSB/(deg/s)
// Output in rad/s
const float GYRO_SCALE = (PI / 180.0f) / 32.8f;

uint32_t lastSampleTime = 0;


// ========================================
// WRITE MPU6050 REGISTER
// ========================================

bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}


// ========================================
// INITIALIZE MPU6050
// ========================================

bool initializeMPU6050() {

    // Wake up MPU6050
    // Use gyro PLL clock
    if (!writeRegister(0x6B, 0x01))
        return false;

    delay(100);

    // Digital low-pass filter: DLPF_CFG = 3
    if (!writeRegister(0x1A, 0x03))
        return false;

    // Sample rate: 1000 / (1 + 9) = 100 Hz
    if (!writeRegister(0x19, 0x09))
        return false;

    // Gyroscope range: +/- 1000 deg/s
    if (!writeRegister(0x1B, 0x10))
        return false;

    // Accelerometer range: +/- 2g
    if (!writeRegister(0x1C, 0x00))
        return false;

    return true;
}


// ========================================
// READ MPU6050 MEASUREMENTS
// ========================================

bool readIMU(
    float &ax, float &ay, float &az,
    float &gx, float &gy, float &gz
) {

    // First accelerometer register
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0)
        return false;

    // Read 14 consecutive bytes:
    // AX, AY, AZ, TEMP, GX, GY, GZ
    if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, true) != 14)
        return false;

    int16_t rawAX =
        (int16_t)((Wire.read() << 8) | Wire.read());

    int16_t rawAY =
        (int16_t)((Wire.read() << 8) | Wire.read());

    int16_t rawAZ =
        (int16_t)((Wire.read() << 8) | Wire.read());

    // Skip temperature (2 bytes)
    Wire.read();
    Wire.read();

    int16_t rawGX =
        (int16_t)((Wire.read() << 8) | Wire.read());

    int16_t rawGY =
        (int16_t)((Wire.read() << 8) | Wire.read());

    int16_t rawGZ =
        (int16_t)((Wire.read() << 8) | Wire.read());

    // Convert acceleration to m/s^2
    ax = rawAX * ACCEL_SCALE;
    ay = rawAY * ACCEL_SCALE;
    az = rawAZ * ACCEL_SCALE;

    // Convert angular velocity to rad/s
    gx = rawGX * GYRO_SCALE;
    gy = rawGY * GYRO_SCALE;
    gz = rawGZ * GYRO_SCALE;

    return true;
}


// ========================================
// SETUP
// ========================================

void setup() {

    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    delay(500);

    if (!initializeMPU6050()) {

        Serial.println("MPU6050 initialization failed!");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("MPU6050 READY");
    Serial.println("type,t_us,ax,ay,az,gx,gy,gz");

    lastSampleTime = micros();
}


// ========================================
// MAIN LOOP
// ========================================

void loop() {

    uint32_t currentTime = micros();

    // Collect data every 10 ms
    if ((uint32_t)(currentTime - lastSampleTime)
        < SAMPLE_PERIOD_US) {
        return;
    }

    lastSampleTime = currentTime;

    float ax, ay, az;
    float gx, gy, gz;

    if (!readIMU(ax, ay, az, gx, gy, gz)) {
        return;
    }

    // ====================================
    // SEND CSV DATA OVER USB
    // ====================================
    //
    // IMU,t_us,ax,ay,az,gx,gy,gz
    //
    // t_us: microseconds
    // ax,ay,az: m/s^2
    // gx,gy,gz: rad/s
    //

    Serial.print("IMU,");
    Serial.print(currentTime);

    Serial.print(",");
    Serial.print(ax, 6);

    Serial.print(",");
    Serial.print(ay, 6);

    Serial.print(",");
    Serial.print(az, 6);

    Serial.print(",");
    Serial.print(gx, 6);

    Serial.print(",");
    Serial.print(gy, 6);

    Serial.print(",");
    Serial.println(gz, 6);
}
