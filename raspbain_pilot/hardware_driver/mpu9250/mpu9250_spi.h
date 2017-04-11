/*
 * mpu9250_spi.h
 *
 *  Created on: Nov 6, 2015
 *      Author: zz269
 */

#ifndef MPU9250_SPI_H_
#define MPU9250_SPI_H_

#include"my_types.h"

#define MPU9250_SPI_RP3 "/dev/spidev0.0"

#define MPU9250_Device_ID           ((u8)0x71)
#define MPU9250_SELF_TEST_XG        ((u8)0x00)
#define MPU9250_SELF_TEST_YG        ((u8)0x01)
#define MPU9250_SELF_TEST_ZG        ((u8)0x02)
#define MPU9250_SELF_TEST_XA        ((u8)0x0D)
#define MPU9250_SELF_TEST_YA        ((u8)0x0E)
#define MPU9250_SELF_TEST_ZA        ((u8)0x0F)
#define MPU9250_XG_OFFSET_H         ((u8)0x13)
#define MPU9250_XG_OFFSET_L         ((u8)0x14)
#define MPU9250_YG_OFFSET_H         ((u8)0x15)
#define MPU9250_YG_OFFSET_L         ((u8)0x16)
#define MPU9250_ZG_OFFSET_H         ((u8)0x17)
#define MPU9250_ZG_OFFSET_L         ((u8)0x18)
#define MPU9250_SMPLRT_DIV          ((u8)0x19)
#define MPU9250_CONFIG              ((u8)0x1A)
#define MPU9250_GYRO_CONFIG         ((u8)0x1B)
#define MPU9250_ACCEL_CONFIG        ((u8)0x1C)
#define MPU9250_ACCEL_CONFIG_2      ((u8)0x1D)
#define MPU9250_LP_ACCEL_ODR        ((u8)0x1E)
#define MPU9250_MOT_THR             ((u8)0x1F)
#define MPU9250_FIFO_EN             ((u8)0x23)
#define MPU9250_I2C_MST_CTRL        ((u8)0x24)
#define MPU9250_I2C_SLV0_ADDR       ((u8)0x25)
#define MPU9250_I2C_SLV0_REG        ((u8)0x26)
#define MPU9250_I2C_SLV0_CTRL       ((u8)0x27)
#define MPU9250_I2C_SLV1_ADDR       ((u8)0x28)
#define MPU9250_I2C_SLV1_REG        ((u8)0x29)
#define MPU9250_I2C_SLV1_CTRL       ((u8)0x2A)
#define MPU9250_I2C_SLV2_ADDR       ((u8)0x2B)
#define MPU9250_I2C_SLV2_REG        ((u8)0x2C)
#define MPU9250_I2C_SLV2_CTRL       ((u8)0x2D)
#define MPU9250_I2C_SLV3_ADDR       ((u8)0x2E)
#define MPU9250_I2C_SLV3_REG        ((u8)0x2F)
#define MPU9250_I2C_SLV3_CTRL       ((u8)0x30)
#define MPU9250_I2C_SLV4_ADDR       ((u8)0x31)
#define MPU9250_I2C_SLV4_REG        ((u8)0x32)
#define MPU9250_I2C_SLV4_DO         ((u8)0x33)
#define MPU9250_I2C_SLV4_CTRL       ((u8)0x34)
#define MPU9250_I2C_SLV4_DI         ((u8)0x35)
#define MPU9250_I2C_MST_STATUS      ((u8)0x36)
#define MPU9250_INT_PIN_CFG         ((u8)0x37)
#define MPU9250_INT_ENABLE          ((u8)0x38)
#define MPU9250_INT_STATUS          ((u8)0x3A)
#define MPU9250_ACCEL_XOUT_H        ((u8)0x3B)
#define MPU9250_ACCEL_XOUT_L        ((u8)0x3C)
#define MPU9250_ACCEL_YOUT_H        ((u8)0x3D)
#define MPU9250_ACCEL_YOUT_L        ((u8)0x3E)
#define MPU9250_ACCEL_ZOUT_H        ((u8)0x3F)
#define MPU9250_ACCEL_ZOUT_L        ((u8)0x40)
#define MPU9250_TEMP_OUT_H          ((u8)0x41)
#define MPU9250_TEMP_OUT_L          ((u8)0x42)
#define MPU9250_GYRO_XOUT_H         ((u8)0x43)
#define MPU9250_GYRO_XOUT_L         ((u8)0x44)
#define MPU9250_GYRO_YOUT_H         ((u8)0x45)
#define MPU9250_GYRO_YOUT_L         ((u8)0x46)
#define MPU9250_GYRO_ZOUT_H         ((u8)0x47)
#define MPU9250_GYRO_ZOUT_L         ((u8)0x48)
#define MPU9250_EXT_SENS_DATA_00    ((u8)0x49)
#define MPU9250_EXT_SENS_DATA_01    ((u8)0x4A)
#define MPU9250_EXT_SENS_DATA_02    ((u8)0x4B)
#define MPU9250_EXT_SENS_DATA_03    ((u8)0x4C)
#define MPU9250_EXT_SENS_DATA_04    ((u8)0x4D)
#define MPU9250_EXT_SENS_DATA_05    ((u8)0x4E)
#define MPU9250_EXT_SENS_DATA_06    ((u8)0x4F)
#define MPU9250_EXT_SENS_DATA_07    ((u8)0x50)
#define MPU9250_EXT_SENS_DATA_08    ((u8)0x51)
#define MPU9250_EXT_SENS_DATA_09    ((u8)0x52)
#define MPU9250_EXT_SENS_DATA_10    ((u8)0x53)
#define MPU9250_EXT_SENS_DATA_11    ((u8)0x54)
#define MPU9250_EXT_SENS_DATA_12    ((u8)0x55)
#define MPU9250_EXT_SENS_DATA_13    ((u8)0x56)
#define MPU9250_EXT_SENS_DATA_14    ((u8)0x57)
#define MPU9250_EXT_SENS_DATA_15    ((u8)0x58)
#define MPU9250_EXT_SENS_DATA_16    ((u8)0x59)
#define MPU9250_EXT_SENS_DATA_17    ((u8)0x5A)
#define MPU9250_EXT_SENS_DATA_18    ((u8)0x5B)
#define MPU9250_EXT_SENS_DATA_19    ((u8)0x5C)
#define MPU9250_EXT_SENS_DATA_20    ((u8)0x5D)
#define MPU9250_EXT_SENS_DATA_21    ((u8)0x5E)
#define MPU9250_EXT_SENS_DATA_22    ((u8)0x5F)
#define MPU9250_EXT_SENS_DATA_23    ((u8)0x60)
#define MPU9250_I2C_SLV0_DO         ((u8)0x63)
#define MPU9250_I2C_SLV1_DO         ((u8)0x64)
#define MPU9250_I2C_SLV2_DO         ((u8)0x65)
#define MPU9250_I2C_SLV3_DO         ((u8)0x66)
#define MPU9250_I2C_MST_DELAY_CTRL  ((u8)0x67)
#define MPU9250_SIGNAL_PATH_RESET   ((u8)0x68)
#define MPU9250_MOT_DETECT_CTRL     ((u8)0x69)
#define MPU9250_USER_CTRL           ((u8)0x6A)
#define MPU9250_PWR_MGMT_1          ((u8)0x6B)
#define MPU9250_PWR_MGMT_2          ((u8)0x6C)
#define MPU9250_FIFO_COUNTH         ((u8)0x72)
#define MPU9250_FIFO_COUNTL         ((u8)0x73)
#define MPU9250_FIFO_R_W            ((u8)0x74)
#define MPU9250_WHO_AM_I            ((u8)0x75)	// ID = 0x71 In MPU9250
#define MPU9250_XA_OFFSET_H         ((u8)0x77)
#define MPU9250_XA_OFFSET_L         ((u8)0x78)
#define MPU9250_YA_OFFSET_H         ((u8)0x7A)
#define MPU9250_YA_OFFSET_L         ((u8)0x7B)
#define MPU9250_ZA_OFFSET_H         ((u8)0x7D)
#define MPU9250_ZA_OFFSET_L         ((u8)0x7E)

/* FIFO enable for storing different values */
#define MPU9250_FIFO_TEMP_OUT        0x80
#define MPU9250_FIFO_GYRO_X_OUT      0x40
#define MPU9250_FIFO_GYRO_Y_OUT      0x20
#define MPU9250_FIFO_GYRO_Z_OUT      0x10
#define MPU9250_ACCEL_OUT            0x08

/* Interrupt Configuration */
#define MPU9250_INT_ACTL             0x80
#define MPU9250_INT_OPEN             0x40
#define MPU9250_INT_LATCH_EN         0x20
#define MPU9250_INT_CLR_ANYRD        0x10
#define MPU9250_INT_I2C_BYPASS_EN    0x02

#define MPU9250_INTEN_OVERFLOW       0x10
#define MPU9250_INTEN_DATA_RDY       0x01

/* Interrupt status */
#define MPU9250_INT_STATUS_OVERFLOW  0x10
#define MPU9250_INT_STATUS_IMU_RDY   0X04
#define MPU9250_INT_STATUS_DATA_RDY  0X01

/* User control functionality */
#define MPU9250_USERCTL_FIFO_EN      0X40
#define MPU9250_USERCTL_I2C_MST_EN   0X20
#define MPU9250_USERCTL_DIS_I2C      0X10
#define MPU9250_USERCTL_FIFO_RST     0X04
#define MPU9250_USERCTL_I2C_MST_RST  0X02
#define MPU9250_USERCTL_GYRO_RST     0X01

/* Power management and clock selection */
#define MPU9250_PWRMGMT_IMU_RST      0X80
#define MPU9250_PWRMGMT_INTERN_CLK   0X00
#define MPU9250_PWRMGMT_PLL_X_CLK    0X01
#define MPU9250_PWRMGMT_PLL_Y_CLK    0X02
#define MPU9250_PWRMGMT_PLL_Z_CLK    0X03
#define MPU9250_PWRMGMT_STOP_CLK     0X07

#define MPUREG_SMPLRT_1000HZ         0x00
#define MPUREG_SMPLRT_500HZ          0x01
#define MPUREG_SMPLRT_250HZ          0x03
#define MPUREG_SMPLRT_200HZ          0x04
#define MPUREG_SMPLRT_100HZ          0x09
#define MPUREG_SMPLRT_50HZ           0x13

enum MPU9250_range {
	MPU9250_SCALE_250_DEG  = 0x00,
	MPU9250_SCALE_500_DEG  = 0x08,
	MPU9250_SCALE_1000_DEG = 0x10,
	MPU9250_SCALE_2000_DEG = 0x18
};
enum MPU9250_accel_range {
	MPU9250_ACCEL_2G = 0x00,
	MPU9250_ACCEL_4G = 0x08,
	MPU9250_ACCEL_8G = 0x10,
	MPU9250_ACCEL_16G = 0x18
};
enum mpu9250_gyro_filter {
	MPU9250_GYRO_LOWPASS_250_HZ = 0x00,
	MPU9250_GYRO_LOWPASS_184_HZ = 0x01,
	MPU9250_GYRO_LOWPASS_92_HZ  = 0x02,
	MPU9250_GYRO_LOWPASS_41_HZ  = 0x03,
	MPU9250_GYRO_LOWPASS_20_HZ  = 0x04,
	MPU9250_GYRO_LOWPASS_10_HZ  = 0x05,
	MPU9250_GYRO_LOWPASS_5_HZ   = 0x06,
	MPU9250_GYRO_LOWPASS		= 0x07
};

enum mpu9250_accel_filter {
	MPU9250_ACCEL_LOWPASS_460_HZ = 0x00,
	MPU9250_ACCEL_LOWPASS_184_HZ = 0x01,
	MPU9250_ACCEL_LOWPASS_92_HZ  = 0x02,
	MPU9250_ACCEL_LOWPASS_41_HZ  = 0x03,
	MPU9250_ACCEL_LOWPASS_20_HZ  = 0x04,
	MPU9250_ACCEL_LOWPASS_10_HZ  = 0x05,
	MPU9250_ACCEL_LOWPASS_5_HZ   = 0x06
};

enum mpu9250_orientation { // clockwise rotation from board forward
	MPU9250_TOP_0DEG       = 0x00,
	MPU9250_TOP_90DEG      = 0x01,
	MPU9250_TOP_180DEG     = 0x02,
	MPU9250_TOP_270DEG     = 0x03,
	MPU9250_BOTTOM_0DEG    = 0x04,
	MPU9250_BOTTOM_90DEG   = 0x05,
	MPU9250_BOTTOM_180DEG  = 0x06,
	MPU9250_BOTTOM_270DEG  = 0x07
};

// The SPI Mode
enum SPIMODE{
	MODE0 = 0,   //!< Low at idle, capture on rising clock edge
	MODE1 = 1,   //!< Low at idle, capture on falling clock edge
	MODE2 = 2,   //!< High at idle, capture on falling clock edge
	MODE3 = 3    //!< High at idle, capture on rising clock edge
};
//The SPI Property
struct mpu9250_spi_property{
	int fd;
	u8  bits;
	u32 speed;
	int mode;
	char *dev;
};
struct mpu9250_spi_SensorData{
	float accel_x;
	float accel_y;
	float accel_z;
	float temp;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	u16 poll_data_count;
};

int mpu9250_spi_open(struct mpu9250_spi_property *mpu9250_property);
void mpu9250_spi_close(struct mpu9250_spi_property *mpu9250_property);
int mpu9250_spi_writeRegister(struct mpu9250_spi_property *mpu9250_property,unsigned int registerAddress, unsigned char value);
int mpu9250_spi_readRegister(struct mpu9250_spi_property *mpu9250_property, unsigned int registerAddress, unsigned char *registerValue);
int mpu9250_spi_readRegisters(struct mpu9250_spi_property *mpu9250_property, unsigned int number, unsigned int fromAddress, unsigned char *registerValue);


#endif /* MPU9250_SPI_H_ */
