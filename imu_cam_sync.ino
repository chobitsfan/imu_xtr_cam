#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

typedef struct __attribute__((packed)) {
  uint32_t ts;
  uint8_t xtr;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} Payload;

// Create a new sensor object
BMI270 imu;

// SPI parameters
const uint8_t chipSelectPin = 5;
const uint32_t clockFrequency = 100000;

// Pin used for interrupt detection
const uint8_t interruptPin = 3;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

unsigned long exp_ts = 0;
unsigned int cnt = 0;

// CRC16-CCITT (0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMI270 Example 4 - Filtering");

    Serial1.begin(921600);

    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);

    // Initialize the SPI library
    SPI.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x68)
    while(imu.beginSPI(chipSelectPin, clockFrequency) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMI270 connected!");

    // The accelerometer and gyroscope can be configured with multiple settings
    // to reduce the measurement noise. Both sensors have the following settings
    // in common:
    // .range       - Measurement range. Lower values give more resolution, but
    //                doesn't affect noise significantly, and limits the max
    //                measurement before saturating the sensor
    // .odr         - Output data rate in Hz. Lower values result in less noise,
    //                but lower sampling rates.
    // .filter_perf - Filter performance mode. Performance oprtimized mode
    //                results in less noise, but increased power consumption
    // .bwp         - Filter bandwidth parameter. This has several possible
    //                settings that can reduce noise, but cause signal delay
    // 
    // Both sensors have different possible values for each setting:
    // 
    // Accelerometer values:
    // .range       - 2g to 16g
    // .odr         - Depends on .filter_perf:
    //                  Performance mode: 12.5Hz to 1600Hz
    //                  Power mode:       0.78Hz to 400Hz
    // .bwp         - Depends on .filter_perf:
    //                  Performance mode: Normal, OSR2, OSR4, CIC
    //                  Power mode:       Averaging from 1 to 128 samples
    // 
    // Gyroscope values:
    // .range       - 125dps to 2000dps (deg/sec)
    // .ois_range   - 250dps or 2000dps (deg/sec) Only relevant when using OIS,
    //                see datasheet for more info. Defaults to 250dps
    // .odr         - Depends on .filter_perf:
    //                  Performance mode: 25Hz to 3200Hz
    //                  Power mode:       25Hz to 100Hz
    // .bwp         - Normal, OSR2, OSR4, CIC
    // .noise_perf  - Similar to .filter_perf. Performance oprtimized mode
    //                results in less noise, but increased power consumption
    // 
    // Note that not all combinations of values are possible. The performance
    // mode restricts which ODR settings can be used, and the ODR restricts some
    // bandwidth parameters. An error code is returned by setConfig, which can
    // be used to determine whether the selected settings are valid.
    int8_t err = BMI2_OK;

    // Set accelerometer config
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    accelConfig.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range = BMI2_ACC_RANGE_8G;
    err = imu.setConfig(accelConfig);

    // Set gyroscope config
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(gyroConfig);

    // Check whether the config settings above were valid
    while(err != BMI2_OK)
    {
        // Not valid, determine which config was the problem
        if(err == BMI2_E_ACC_INVALID_CFG)
        {
            Serial.println("Accelerometer config not valid!");
        }
        else if(err == BMI2_E_GYRO_INVALID_CFG)
        {
            Serial.println("Gyroscope config not valid!");
        }
        else if(err == BMI2_E_ACC_GYR_INVALID_CFG)
        {
            Serial.println("Both configs not valid!");
        }
        else
        {
            Serial.print("Unknown error: ");
            Serial.println(err);
        }
        delay(1000);
    }

    imu.mapInterruptToPin(BMI2_DRDY_INT, BMI2_INT1);

    bmi2_int_pin_config intPinConfig;
    intPinConfig.pin_type = BMI2_INT1;
    intPinConfig.int_latch = BMI2_INT_NON_LATCH;
    intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    imu.setInterruptPinConfig(intPinConfig);

    attachInterrupt(digitalPinToInterrupt(interruptPin), myInterruptHandler, RISING);

    Serial.println("Configuration valid! Beginning measurements");
    delay(1000);
}

void loop()
{
    if(interruptOccurred) {
        interruptOccurred = false;
        uint16_t interruptStatus = 0;
        imu.getInterruptStatus(&interruptStatus);
        if((interruptStatus & BMI2_GYR_DRDY_INT_MASK) && (interruptStatus & BMI2_ACC_DRDY_INT_MASK)) {
            Payload dataToSend;
            dataToSend.xtr = 0;
            uint32_t ts = micros();
            cnt++;
            if (cnt > 9) {
                cnt = 0;
                digitalWrite(8, LOW);
                exp_ts = ts;
                dataToSend.xtr = 1;
                //Serial.println("xtr");
            }

            // Get measurements from the sensor. This must be called before accessing
            // the sensor data, otherwise it will never update
            imu.getSensorData();

            dataToSend.ts = ts;
            dataToSend.ax = imu.data.accelX;
            dataToSend.ay = imu.data.accelY;
            dataToSend.az = imu.data.accelZ;
            dataToSend.gx = imu.data.gyroX;
            dataToSend.gy = imu.data.gyroY;
            dataToSend.gz = imu.data.gyroZ;

            uint16_t crc = crc16_ccitt((uint8_t*)&dataToSend, sizeof(Payload));

            Serial1.write(0xaa);
            Serial1.write(0x55);
            Serial1.write((uint8_t*)&dataToSend, sizeof(Payload));
            Serial1.write((uint8_t)(crc >> 8));   // CRC high byte
            Serial1.write((uint8_t)(crc & 0xFF)); // CRC low byte

            // Print acceleration data
            /*Serial.print("Acceleration in g's");
            Serial.print("\t");
            Serial.print("X: ");
            Serial.print(imu.data.accelX, 4);
            Serial.print("\t");
            Serial.print("Y: ");
            Serial.print(imu.data.accelY, 4);
            Serial.print("\t");
            Serial.print("Z: ");
            Serial.print(imu.data.accelZ, 4);

            Serial.print("\t");

            // Print rotation data
            Serial.print("Rotation in deg/sec");
            Serial.print("\t");
            Serial.print("X: ");
            Serial.print(imu.data.gyroX, 3);
            Serial.print("\t");
            Serial.print("Y: ");
            Serial.print(imu.data.gyroY, 3);
            Serial.print("\t");
            Serial.print("Z: ");
            Serial.println(imu.data.gyroZ, 3);*/
        }
    } else {
        if (exp_ts > 0 && (micros() - exp_ts) > 10000) {
            digitalWrite(8, HIGH);
            exp_ts = 0;
        }
    }
    // Print 50x per second
    //delay(20);
}

void myInterruptHandler()
{
    interruptOccurred = true;
}