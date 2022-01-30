#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>

#if defined(__cplusplus)
namespace mpu9250 {
#endif

#define SIGNAL_MPU9250 42

#define IOCTL_MPU9250_MAGIC 0xA8
/* registers the given pid to be interrupted when a bump is detected */
#define IOCTL_MPU9250_REGISTER_PID _IOW(IOCTL_MPU9250_MAGIC, 0, int)
/* retrieves the data currently held in the bump detection buffer,
 * argument has to be a buffer capable of holding 265 mpu9250_data_padded_t
 */
#define IOCTL_MPU9250_GIB_DATA _IOR(IOCTL_MPU9250_MAGIC, 1, long unsigned)

#define IOCTL_MPU9250_GIB_DATA_BLOCK_SIZE 4
#define IOCTL_MPU9250_GIB_DATA_SIZE 256

#pragma pack(push, 1)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Axis_t;

typedef struct {
    Axis_t accel;
    Axis_t gyro;
    Axis_t mag;
} mpu9250_data_t;

typedef struct {
    int64_t timestamp_ns;
    mpu9250_data_t data;
} mpu9250_t;

typedef struct {
	mpu9250_data_t data;
	// we have to read 5x 32bit and this makes it so we can use
	// ioread32_rep(addr, buffer, 5 * 265) function,bc speeeeEEED!
	// (also not every platform allows halfword aligned access,
	// so doing this is a nice exercise in "making things portable")
	uint16_t padding;
} mpu9250_data_padded_t;

typedef struct {
	int64_t timestamp_ns;
	mpu9250_data_padded_t data[IOCTL_MPU9250_GIB_DATA_BLOCK_SIZE][IOCTL_MPU9250_GIB_DATA_SIZE]; // 1024 values total
} bumpData_t;

#pragma pack(pop)

#if defined(__cplusplus)
} // end namespace
#endif

#endif
