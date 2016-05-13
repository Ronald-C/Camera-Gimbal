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


// Accelerometer & Gyroscope Register mapping
#define GYRO_FULL_SCALE_2000_DPS    0x18
#define ACC_FULL_SCALE_16_G         0x18

// << DEVICE ADDRESSES >>
#define    AK8963_ADDRESS      0x0C    // Address of magnetometer
#define    MPU9250_ADDRESS     0xd0    // Device address when AD0 = GND


class MPU9250 : public scheduler_task
{
    public:
        MPU9250(uint8_t priority) : scheduler_task("MPU9250", 512*4, priority)
        {
        }

        bool init(void)
        {
            // Verify device identity
            uint8_t WAI = mpu.readReg(MPU9250_ADDRESS, MPU9250_WAI);
            if(WAI == MPU9250_WhoAmIExpectedValue) {
                u0_dbg_printf("\nI2C Read Test Passed, MPU6500 Address: 0x%x\n", WAI);

            } else {
                u0_dbg_printf("ERROR: I2C MPU6500 Read Test Failed, Stopping\n");
                while(1){}
            }

            // Auto select clock source and Enable Acc & Gyro
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
            mpu.writeReg(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);

            // Acc & Gyro sample configuration
            mpu.writeReg(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FULL_SCALE_2000_DPS);
            mpu.writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, ACC_FULL_SCALE_16_G);

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

        bool run(void *p)
        {
            // Accelerometer
            int16_t AcX = get16BitRegister(ACCEL_XOUT_H);
            int16_t AcY = get16BitRegister(ACCEL_YOUT_H);
            int16_t AcZ = get16BitRegister(ACCEL_ZOUT_H);

            // Gyproscope
            int16_t GyX = get16BitRegister(GYRO_XOUT_H);
            int16_t GyY = get16BitRegister(GYRO_YOUT_H);
            int16_t GyZ = get16BitRegister(GYRO_ZOUT_H);

            double roll = atan2(AcY, AcZ) * 57.2957;
            double pitch = atan2(double(-AcX), sqrt(double(AcY * AcY) + double(AcZ * AcZ)));

            printf("roll: %f pitch %f\n", roll, pitch);
            vTaskDelay(1000);



            return true;
        }

    private:
        I2C2 &mpu = I2C2::getInstance();        // Get I2C bus semaphore

        /// Expected value of Sensor's "WHO AM I" register
        static const unsigned char MPU9250_WhoAmIExpectedValue = 0x71;
        typedef enum {
            GYRO_CONFIG=0x1B, ACCEL_CONFIG=0x1C,

            ACCEL_XOUT_H=0x3B, ACCEL_YOUT_H=0x3D, ACCEL_ZOUT_H=0x3F,
            GYRO_XOUT_H=0x43, GYRO_YOUT_H=0x45, GYRO_ZOUT_H=0x47,

            PWR_MGMT_1=0x6B, PWR_MGMT_2=0x6C,
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
