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
 *
 * The MPU-9150 contains the MPU-6050 acc/gyro and an AK8975 mag. from AKM. The
 * MPU-9250 contains a MPU-6500 and AK8963. We use the MPU-9265 here, which contains
 * a MPU-6515. Code is mostly backwards compatible except for difference in
 * registers present.
 */

#include <math.h>
#include "printf_lib.h"     // u0_dbg_printf()
#include "i2c2.hpp"         // for I2C bus access

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

// << DEVICE ADDRESSES >>
#define    AK8963_ADDRESS      0x0C    // Address of magnetometer
#define    MPU9250_ADDRESS     0xd0    // Device address when AD0 = GND


class MPU9250 : public scheduler_task
{
    public:
        MPU9250(uint8_t priority) : scheduler_task("MPU9250", 512*4, priority)
        {
        }

        void initAK8963(float )
        {

        }

        void verifyMPU9250(void)
        {
            uint8_t WAI = mpu.readReg(MPU9250_ADDRESS, MPU9250_WAI);
            if(WAI == MPU9250_WhoAmIExpectedValue) {
                u0_dbg_printf("\nI2C Read Test Passed, MPU6500 Address: 0x%x\n", WAI);

            } else {
                u0_dbg_printf("ERROR: I2C MPU6500 Read Test Failed, Stopping\n");
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
                    gyroDivider = 131;
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
                    accDivider = 16384;
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

        bool init(void)
        {
            // Initialize MPU9250 device; wake up device
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);

            verifyMPU9250(); // Verify device identity

            // Auto select clock source and Enable Acc & Gyro
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);    // PLL with x-axis gyroscope reference
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);

            mpu.writeReg(MPU9250_ADDRESS, USER_CTRL, 0x20);     // I2C Master mode
            mpu.writeReg(MPU9250_ADDRESS, MST_CTRL, 0x0D);      // 400 kHz ; 20 divider

            // Set accelerometer & gyroscope sensitivity
            setGyroScale(GFS_2000DPS);
            setAccScale(AFS_16G);

            return true;
        }

        /**
         * Reads 16-bit register from reg and reg+1 granted that reg has MSB
         */
        uint16_t get16BitRegister(unsigned char reg)
        {
            uint8_t buff[2] = {0};
            mpu.readRegisters(MPU9250_ADDRESS, reg, &buff[0], sizeof(buff));

            const uint16_t MSB = buff[0];
            const uint16_t LSB = buff[1];
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
        void getAcceleration(int16_t *rax, int16_t *ray, int16_t *raz)
        {
            *rax = get16BitRegister(ACCEL_XOUT_H) / accDivider;
            *ray = get16BitRegister(ACCEL_YOUT_H) / accDivider;
            *raz = get16BitRegister(ACCEL_YOUT_H) / accDivider;
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
        void getRotation(int16_t *rgx, int16_t *rgy, int16_t *rgz)
        {
            *rgx = get16BitRegister(GYRO_XOUT_H) / 16.4;
            *rgy = get16BitRegister(GYRO_YOUT_H) / 16.4;
            *rgz = get16BitRegister(GYRO_ZOUT_H) / 16.4;
        }

        bool run(void *p)
        {
            int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

            getAcceleration(&AcX, &AcY, &AcZ);       // Accelerometer
            getRotation(&GyX, &GyY, &GyZ);           // Gyroscope

            double roll = atan2(AcY, AcZ) * 57.2957;
            double pitch = atan2(double(-AcX), sqrt(double(AcY * AcY) + double(AcZ * AcZ)));

            printf("roll: %f pitch %f\n", roll, pitch);
            printf("gyro %f %f %f\n", GyX, GyY, GyZ);

            vTaskDelay(2000);



            return true;
        }

    private:
        I2C2 &mpu = I2C2::getInstance();        // Get I2C bus semaphore

        float accDivider = 0;
        float gyroDivider = 0;

        /// Expected value of Sensor's "WHO AM I" register
        static const unsigned char MPU9250_WhoAmIExpectedValue = 0x71;
        typedef enum {
            MST_CTRL=0x24, 
            I2C_SLV0_ADDR=0x25, I2C_SLV0_REG=0x26, I2C_SLV0_CTRL=0x27,

            GYRO_CONFIG=0x1B, ACCEL_CONFIG=0x1C,

            ACCEL_XOUT_H=0x3B, ACCEL_YOUT_H=0x3D, ACCEL_ZOUT_H=0x3F,
            GYRO_XOUT_H=0x43, GYRO_YOUT_H=0x45, GYRO_ZOUT_H=0x47,

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

        } __attribute__ ((packed)) AK8963_RegisterMap;

};

#endif /* L5_APPLICATION_GIMBAL_MPU9250_HPP_ */
