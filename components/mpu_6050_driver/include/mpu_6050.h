#ifndef MPU_6050_H
#define MPU_6050_H
#include "i2c_hal.h"

extern hal_i2c_bus_handle_t i2c_handle;

typedef enum{
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    XY_AXIS,
    XZ_AXIS,
    YZ_AXIS,
    ALL_AXIS
}axis_t;
typedef enum{
    BAND_260HZ,
    BAND_184HZ,
    BAND_94HZ,
    BAND_44HZ,
    BAND_21HZ,
    BAND_10HZ,
    BAND_5HZ
}bandwidth_t;


typedef struct{
    uint8_t Data_rdy; // habilita interrupcion de datos listos
    uint8_t int_level; // nivel de interrupcion
    uint8_t sample_rate; // frecuencia de muestreo divide a la frecuencia base
    bandwidth_t bandwidth; //frecuencia de corte del filtro pasa bajo
    
}interrupt_config_t;







void bus_init();
void mpu_6050_init(uint8_t device_address, uint32_t speed, interrupt_config_t *interrup_config);
void mpu_6050_write(uint8_t *data, size_t data_length);
void mpu_6050_read_acceleration(axis_t axis, int16_t *buffer_out);
void mpu_6050_read_gyroscope(axis_t axis, uint16_t *buffer_out);

#endif