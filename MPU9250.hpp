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

#include "printf_lib.h"     // u0_dbg_printf()
#include "i2c2.hpp"         // for I2C bus access

// See MPU-9250 Register Map and Descriptions, Revision 1.4 and MPU-9250 Product
// Specifications Revision 1.0
//
// Magnetometer (AK8963) Register Mapping
// NOTE: RSV (0x0B), TS1 (0x0D), TS2 (0x0E) DO NOT ACCESS
#define AK8963_WIA      0x00    // Device ID: Fixed @ 0x48
#define AK8963_INFO     0x01
#define AK8963_ST1      0x02    // Data ready status bit 0; DATA STATUS
#define AK8963_XOUT_L   0x03    // Measurement data . . .
#define AK8963_XOUT_H   0x04
#define AK8963_YOUT_L   0x05
#define AK8963_YOUT_H   0x06
#define AK8963_ZOUT_L   0x07
#define AK8963_ZOUT_H   0x08
#define AK8963_ST2      0x09    // Overflow bit 3 and data read output status bit 4; DATA STATUS
#define AK8963_CNTL     0x0A    // Bit 3:0 set operation mode and bit 4 for bit-output <<REF DATASHEET to set FUSE>>
#define AK8963_ASTC     0x0C    // Self test control
#define AK8963_I2CDIS   0x0F    // I2C disable
#define AK8963_ASAX     0x10    // X-axis sensitivity adjustment value; FUSE ROM
#define AK8963_ASAY     0x11    // Y-axis sensitivity adjustment value; FUSE ROM
#define AK8963_ASAZ     0x12    // Z-axis sensitivity adjustment value; FUSE ROM

// Gyroscope and Accelerometer Register Mapping
//
//
#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02
#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F
#define XG_OFFSET_H         0x13  // Gyroscope offset registers; Remove DC bias . . .
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SNOKRT_DIV          0x19 // Set sample for Configuration (0x1A)
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E // Set acc. frequency of wake up sampling
#define WOM_THR             0x1F

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24 // I2C Master mode control
#define I2C_SLV0_ADDR       0x25 // I2C slave 0 control . . .
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28 // I2C slave 1 control . . .
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B // I2C slave 2 control . . .
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E // I2C slave 3 control . . .
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31 // I2C slave 4 control . . .
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36 // I2C Master mode status
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38 // I2C enable
#define INT_STATUS          0x3A // I2C status

#define ACCEL_XOUT_H        0x3B // Accelerometer measurements . . .
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41 // Temperature measurements . . .
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43 // Gyroscope measurements . . .
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

// External sensor data
// These registers store data from external sensors <<REF DATASHEET>>
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define I2C_SLV0_DO      0x63 // I2C slave data out registers . . .
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66

#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define FIFO_COUNTH      0x72 // Bits written to FIFO . . .
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74 // FIFO R/W
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77 // For accelerometer offset cancellation . . .
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// << DEVICE ADDRESSES >>
#define    AK8963_ADDRESS       0x0C    // Address of magnetometer
#if AD0
#define    MPU9250_ADDRESS      0x69    // Device address when AD0 = 1
#else
#define    MPU9250_ADDRESS      0x68    // Device address when AD0 = 0
#endif



#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

class MPU9250 : scheduler_task
{
    public:
        MPU9250(uint8_t priority) : scheduler_task("MPU9250", 512*4, priority)
        {
            setRunDuration(100);
        }
        bool init(void)
        {
            u0_dbg_put("MPU9250 init");
            mpu.writeReg(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS)

            return true;
        }
        bool run(void *p)
        {


            return true;
        }

    private:
        I2C2 &mpu = I2C2::getInstance();        // Get I2c singleton driver instance


};

#endif /* L5_APPLICATION_GIMBAL_MPU9250_HPP_ */
