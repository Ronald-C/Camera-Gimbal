/*
 * MPU9250.hpp
 *
 *  Created on: Apr 22, 2016
 *      Author: Ronald Cheng
 */

#ifndef L5_APPLICATION_GIMBAL_MPU9250_HPP_
#define L5_APPLICATION_GIMBAL_MPU9250_HPP_

/**
 * REFERENCE:
 *  -   https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino
 *  -   https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/#docs
 *
 * The MPU-9150 contains the MPU-6050 acc/gyro and an AK8975 mag. from AKM. The
 * MPU-9250 contains a MPU-6500 and AK8963. We use the MPU-9250 here, which contains
 * a MPU-6500. Code is mostly backwards compatible except for minor differences in
 * registers present.
 */

#include <math.h>
#include "printf_lib.h"     // u0_dbg_printf()
#include "i2c2.hpp"         // for I2C bus access

/// IDs used for getSharedObject() and addSharedObject()
typedef enum {
    shared_CompassID,
} sharedHandleId_t;

// See MPU-9250 Register Map and Descriptions, Revision 1.4 and MPU-9250 Product
// Specifications Revision 1.0
//
// Magnetometer (AK8963) Register Mapping
// NOTE: RSV (0x0B), TS1 (0x0D), TS2 (0x0E) DO NOT ACCESS

// Set initial input parameters
enum Ascale {
  AFS_2G = 0x00,
  AFS_4G = 0x08,
  AFS_8G = 0x10,
  AFS_16G = 0x18
};

enum Gscale {
  GFS_250DPS = 0x00,
  GFS_500DPS = 0x08,
  GFS_1000DPS = 0x10,
  GFS_2000DPS = 0x18
};

const float alpha = 0.01;

// << DEVICE ADDRESSES >>
#define AK8963_ADDRESS      0x0C    // Address of magnetometer
#define MPU9250_ADDRESS     0xd0    // Device address when AD0 = GND

#define AK8963_READ_BYTE    0x80
#define Pi                  3.14159

class MPU9250 : public scheduler_task
{
    public:
        MPU9250(uint8_t priority) : scheduler_task("MPU9250", 512*4, priority)
        {
            magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
            magbias[1] = +120.;  // User environmental y-axis correction in milliGauss
            magbias[2] = +125;   // User environmental z-axis correction in milliGauss
        }

        void verifyMPU9250(void)
        {
            uint8_t WAI = mpu.readReg(MPU9250_ADDRESS, MPU9250_WAI);
            if(WAI == MPU9250_WhoAmIExpectedValue) {
                u0_dbg_printf("\nI2C Read Test Passed, MPU9250 Address: 0x%x\n", WAI);

            } else {
                u0_dbg_printf("ERROR: I2C MPU9250 Read Test Failed, Stopping, 0x%x\n", WAI);
                while(1){}  // Infinite loop
            }
        }

        void setGyroScale(Gscale gyroFS)
        {
            mpu.writeReg(MPU9250_ADDRESS, GYRO_CONFIG, gyroFS);
            switch (gyroFS)
            {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
                case GFS_250DPS:
                    gyroDivider = 131;  // LSB / (deg/s)
                    break;
                case GFS_500DPS:
                    gyroDivider = 65.5;
                    break;
                case GFS_1000DPS:
                    gyroDivider = 32.8;
                    break;
                case GFS_2000DPS:
                    gyroDivider = 16.4;
                    break;
            }
        }

        void setAccScale(Ascale accFS)
        {
            mpu.writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, accFS);
            switch (accFS){
                case AFS_2G:
                    accDivider = 16384;     // LSB / g
                    break;
                case AFS_4G:
                    accDivider = 8192;
                    break;
                case AFS_8G:
                    accDivider = 4096;
                    break;
                case AFS_16G:
                    accDivider = 2048;
                    break;
            }
        }

        void verifyAK8963(void)
        {
            configAK8963(AK8963_ADDRESS | AK8963_READ_BYTE, 0x00, 0x81);
            uint8_t WAI = mpu.readReg(MPU9250_ADDRESS, EXT_SENS_DATA_00);
            if(WAI == AK8963_WhoAmIExpectedValue) {
                u0_dbg_printf("\nI2C Read Test Passed, AK8963 Address: 0x%x\n", WAI);

            } else {
                u0_dbg_printf("ERROR: I2C AK8963 Read Test Failed, Stopping, 0x%x\n", WAI);
                while(1){}  // Infinite loop
            }
        }

        void configAK8963(uint8_t slaveADDR, uint8_t startReg, uint8_t bytesToRead)
        {
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, slaveADDR);    // R/W + SLV_ADDR
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_REG, startReg);
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, bytesToRead);
        }

        bool init(void)
        {
            // Initialize MPU9250 device; wake up device
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);

            mpu.writeReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz

            verifyMPU9250(); // Verify device identity

            // Auto select clock source and Enable Acc & Gyro
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);    // PLL with x-axis gyroscope reference
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);

            mpu.writeReg(MPU9250_ADDRESS, USER_CTRL, 0x20);     // I2C Master mode
            mpu.writeReg(MPU9250_ADDRESS, MST_CTRL, 0x0D);

            // Set accelerometer & gyroscope sensitivity
            setGyroScale(GFS_500DPS);
            setAccScale(AFS_2G);

            // Initialize AK8963 device
            configAK8963(AK8963_ADDRESS, AK8963_CNTL2, 0x81);
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_DO, 0x01);   // reset device
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);
            for(int i=0; i<1000;i++) {};

            //verifyAK8963();

            configAK8963(AK8963_ADDRESS, AK8963_CNTL1, 0x81);
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_DO, 0x12);   // 16-bit output + "0010": Continuous measurement mode 1
            mpu.writeReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);

            getCompass_calib();

            /* Create queue and save shared handle by addSharedObject() */
           QueueHandle_t obj = xQueueCreate(1, sizeof(float));
           addSharedObject(shared_CompassID, obj);

            return true;
        }

        /**
         * Reads 16-bit register from reg and reg+1 granted that reg has MSB
         */
        int16_t get16BitRegister(unsigned char reg)
        {
            uint8_t buff[2] = {0};
            mpu.readRegisters(MPU9250_ADDRESS, reg, &buff[0], sizeof(buff));

            const int16_t MSB = buff[0];
            const int16_t LSB = buff[1];
            return ((MSB << 8) | (LSB & 0xFF));
        }

        /*
         * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
         * (Register 28). For each full scale setting, the accelerometers' sensitivity
         * per LSB in ACCEL_xOUT is shown in the table below:
         *
         * <pre>
         * AFS_SEL | Full Scale Range | LSB Sensitivity
         * --------+------------------+----------------
         * 0       | +/- 2g           | 8192 LSB/mg
         * 1       | +/- 4g           | 4096 LSB/mg
         * 2       | +/- 8g           | 2048 LSB/mg
         * 3       | +/- 16g          | 1024 LSB/mg
         * </pre>
         */
        void getAcceleration(float *angleX, float *angleY)
        {
            int16_t readData[3];
            readData[0] = get16BitRegister(ACCEL_XOUT_H);
            readData[1] = get16BitRegister(ACCEL_YOUT_H);
            readData[2] = get16BitRegister(ACCEL_YOUT_H);

            //Low Pass Filter
            fXg = readData[0] * alpha + (fXg * (1.0 - alpha));
            fYg = readData[1] * alpha + (fYg * (1.0 - alpha));
            fZg = readData[2] * alpha + (fZg * (1.0 - alpha));

            // NOTE: atan2 relies on ratio thus sensitivities may not be needed
            *angleX = 57.295 * atan2f((float)readData[0], sqrt(pow((float)readData[2], 2) + pow((float)readData[1], 2)));   // roll
            *angleY = 57.295 * atan2f((float)-readData[1], (float)readData[2]);                                             // pitch
        }

        /*
         * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
         * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
         * LSB in GYRO_xOUT is shown in the table below:
         *
         * <pre>
         * FS_SEL | Full Scale Range   | LSB Sensitivity
         * -------+--------------------+----------------
         * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
         * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
         * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
         * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
         * </pre>
         */
        void getRotation(float *rgx, float *rgy, float *rgz)
        {
            *rgx = get16BitRegister(GYRO_XOUT_H) / gyroDivider;
            *rgy = get16BitRegister(GYRO_YOUT_H) / gyroDivider;
            *rgz = get16BitRegister(GYRO_ZOUT_H) / gyroDivider;
        }

        void getCompass_calib(void)
        {
            int8_t response[3];
            configAK8963(AK8963_ADDRESS | AK8963_READ_BYTE, AK8963_ASAX, 0x83);

            //mpu.readRegisters(MPU9250_ADDRESS, EXT_SENS_DATA_00, response, (uint32_t)sizeof(response));

            response[0] = get16BitRegister(EXT_SENS_DATA_00);
            response[1] = get16BitRegister(EXT_SENS_DATA_02);
            response[2] = get16BitRegister(EXT_SENS_DATA_04);
            for(int i = 0; i < 3; i++) {
                magnetometer_ASA[i] = (((response[i] - 128) / 256) + 1);
            }
            // Manufacturer sensitivity adjustment values
            u0_dbg_printf("ASAX: %f, ASAY: %f, ASAZ: %f\n", magnetometer_ASA[0], magnetometer_ASA[1], magnetometer_ASA[2]);
        }

        void getCompass(float *gyroDegree)
        {
            // Read AK8963 register 0x03; We read 7 bytes so upon read of ST2 register 0x09,
            // the device will unlatch the data registers for next measurement read
            configAK8963(AK8963_ADDRESS | AK8963_READ_BYTE, AK8963_XOUT_L, 0x87);

            float mRes = 10.*4912./32760.0; // Proper scale to return milliGauss

            int16_t readMag[3];
            gyroDegree[0] = (float)get16BitRegister(EXT_SENS_DATA_00) * magnetometer_ASA[0] * mRes - magbias[0];
            gyroDegree[1] = (float)get16BitRegister(EXT_SENS_DATA_02) * magnetometer_ASA[1] * mRes - magbias[1];
            gyroDegree[2] = (float)get16BitRegister(EXT_SENS_DATA_04) * magnetometer_ASA[2] * mRes - magbias[2];
/*
            //Low Pass Filter
            rgx = readMag[0] * alpha + (rgx * (1.0 - alpha));
            rgy = readMag[1] * alpha + (rgy * (1.0 - alpha));
            rgz = readMag[2] * alpha + (rgz * (1.0 - alpha));


            float tempDegree = ((atan2f(-rgy , rgx) * 180) / Pi ) + declinationAngle;

            if(tempDegree < 0) {
                tempDegree = 360 + tempDegree;
            }
            if(tempDegree > 360) {
                tempDegree = tempDegree - 360;
            }

            *gyroDegree = tempDegree;*/
        }

        bool run(void *p)
        {
            //getAcceleration(&acc_data[0], &acc_data[1]);                        // Accelerometer
            getRotation(&gyro_data[0], &gyro_data[1], &gyro_data[2]);           // Gyroscope                                             // Magnetometer

            //u0_dbg_printf(" mag: %f %f %f\n", mag_data[0], mag_data[1], mag_data[2]);

            SoftTimer t;
            if(sampling && !t.isRunning()) {
                t.reset(48);
                getCompass(mag_data);
                gyro_data[2] /= gyroDivider;

                sampling = false;

            } else {
                yaw = yaw + (gyro_data[2] * 0.048);
                //u0_dbg_printf("yaw %f\n", yaw);

                xQueueSend(getSharedObject(shared_CompassID), &yaw, portMAX_DELAY);
                sampling = true;
            }

            //vTaskDelay(1);

            return true;
        }

    private:
        float a = 0;

        I2C2 &mpu = I2C2::getInstance();        // Get I2C bus semaphore
        bool sampling = true;
        float yaw = 0;

        double fXg = 0, fYg = 0, fZg = 0;
        float rgx = 0, rgy = 0, rgz = 0;

        float magbias[3];
        float declinationAngle = 13.53;            // Declination @ San Jose

        float acc_data[2];
        float gyro_data[3];
        float mag_data[3];

        float magnetometer_ASA[3];
        float accDivider = -1;
        float gyroDivider = -1;

        /// Expected value of Sensor's "WHO AM I" register
        static const unsigned char MPU9250_WhoAmIExpectedValue = 0x71;
        typedef enum {
            SMPLRT_DIV=0x19, MST_CTRL=0x24,
            I2C_SLV0_ADDR=0x25, I2C_SLV0_REG=0x26, I2C_SLV0_CTRL=0x27,
            I2C_SLV1_ADDR=0x28, I2C_SLV1_REG=0x29, I2C_SLV1_CTRL=0x2A,

            GYRO_CONFIG=0x1B, ACCEL_CONFIG=0x1C,

            ACCEL_XOUT_H=0x3B, ACCEL_YOUT_H=0x3D, ACCEL_ZOUT_H=0x3F,
            GYRO_XOUT_H=0x43, GYRO_YOUT_H=0x45, GYRO_ZOUT_H=0x47,

            EXT_SENS_DATA_00=0x49, EXT_SENS_DATA_02=0x4B, EXT_SENS_DATA_04=0x4D,
            I2C_SLV0_DO=0x63,

            USER_CTRL=0x6A, PWR_MGMT_1=0x6B, PWR_MGMT_2=0x6C,
            MPU9250_WAI=0x75,

        } __attribute__ ((packed)) MPU9250_RegisterMap;

        /// Expected value of Sensor's "WHO AM I" register
        static const unsigned char AK8963_WhoAmIExpectedValue = 0x48;
        typedef enum {
            AK8963_WIA=0x00,
            AK8963_INFO=0x01, AK8963_ST1=0x02,

            AK8963_XOUT_L=0x03, AK8963_XOUT_H=0x04, AK8963_YOUT_L=0x05,
            AK8963_YOUT_H=0x06, AK8963_ZOUT_L=0x07, AK8963_ZOUT_H=0x08,

            AK8963_CNTL1=0x0A, AK8963_CNTL2=0x0B,

            AK8963_ASAX=0x10, AK8963_ASAY=0x11, AK8963_ASAZ=0x12,

        } __attribute__ ((packed)) AK8963_RegisterMap;

};

#endif /* L5_APPLICATION_GIMBAL_MPU9250_HPP_ */
