#ifndef I2C_HAL_H
#define I2C_HAL_H
#include "stdint.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
 
#define I2C_MASTER_SCL_IO           22          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0

typedef void* hal_i2c_bus_handle_t;
typedef void* hal_i2c_device_handle_t;


typedef struct {
    int8_t i2c_port;              /*!< I2C port number, `-1` for auto selecting, (not include LP I2C instance) */
    uint8_t sda_io_num;                /*!< GPIO number of I2C SDA signal, pulled-up internally */
    uint8_t scl_io_num;                /*!< GPIO number of I2C SCL signal, pulled-up internally */
}hal_i2c_data_config_t;

typedef struct {
    uint32_t scl_speed_hz;                      /*!< I2C SCL line frequency. */
    uint16_t device_address;                    /*!< I2C device raw address. (The 7/10 bit address without read/write bit) */
}hal_i2c_device_bus_master_config_t;

typedef enum {
    I2C_OK = 0,
    I2C_ERR_INVALID_ARG = 1,
    I2C_ERR_NOT_FOUND = 2,
    I2C_ERR_FAIL = 3
} i2c_error_t;




//bus maestro--------- funciones para manejar el bus i2c como maestro y realizar operaciones de lectura y escritura,etc.
i2c_error_t hal_i2c_master_init(hal_i2c_data_config_t *config, hal_i2c_bus_handle_t *bus_handle); //

i2c_error_t hal_i2c_add_device(hal_i2c_device_bus_master_config_t *device_config, hal_i2c_device_handle_t *device_handle, hal_i2c_bus_handle_t bus_handle);//

i2c_error_t hal_i2c_master_deinit(void);

i2c_error_t hal_i2c_master_write( uint8_t *data, size_t data_length, hal_i2c_device_handle_t device_handle);//

i2c_error_t hal_i2c_master_read_register( uint8_t dir_register, uint8_t *data_read, size_t read_size,hal_i2c_device_handle_t device_handle);


#endif 