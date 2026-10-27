#include "i2c_hal.h"
#include "esp_log.h"

#define TIMEOUT_MS 100 //NO SE PONE -1 PARA NO ESPERAR INFINITAMENTE, SE PONE UN TIEMPO DE ESPERA DE 100MS

///****************FUNCIONES DEL BUS MAESTRO**************************** */


//INICIALIZAR EL BUS MAESTRO
i2c_error_t hal_i2c_master_init(hal_i2c_data_config_t *config, hal_i2c_bus_handle_t *bus_handle)
{
    // 1. Validar los punteros principales PRIMERO
    if (config == NULL || bus_handle == NULL)
    {
        return I2C_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->sda_io_num,
        .scl_io_num = config->scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT, // Recomendado en ESP-IDF v5+
        .glitch_ignore_cnt = 7,            // Valor estándar de filtrado
        .flags.enable_internal_pullup = true// Opcional, actívalo si tus módulos no tienen pull-ups físicos
    };

    i2c_master_bus_handle_t handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &handle);
    
    if (err != ESP_OK)
    {
        return I2C_ERR_FAIL;
    }

    // 3. Asignar de forma segura (ya validamos que bus_handle no es nulo)
    *bus_handle = handle;

    return I2C_OK;
}

//****************INSTALAR UN DISPOSITIVO AL BUS MAESTRO*************************** */

i2c_error_t hal_i2c_add_device(hal_i2c_device_bus_master_config_t *device_config, hal_i2c_device_handle_t *device_handle, hal_i2c_bus_handle_t bus_handle)
{
    if (device_config == NULL)
    {
        return I2C_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7, // Asumiendo dirección de 7 bits
        .device_address = device_config->device_address,
        .scl_speed_hz = device_config->scl_speed_hz,
        .flags.disable_ack_check = false // Habilitar verificación de ACK
    };
    i2c_master_dev_handle_t dev_handle;
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    
    if (err != ESP_OK)
    {
        return I2C_ERR_FAIL;
    }

    *device_handle = dev_handle;
    return I2C_OK;
}


//****************FUNCION DE ESCRITURA**************************** */
i2c_error_t hal_i2c_master_write(uint8_t *data, size_t data_length, hal_i2c_device_handle_t device_handle)
{
    esp_err_t err = i2c_master_transmit(device_handle, data, data_length, TIMEOUT_MS);
    if (err != ESP_OK)
    {
        return I2C_ERR_FAIL;
    }

    return I2C_OK; // Retorna I2C_OK si la operación fue exitosa
}

//****************F**************************** */
i2c_error_t hal_i2c_master_read_register( uint8_t dir_register, uint8_t *data_read, size_t read_size,hal_i2c_device_handle_t device_handle)


{
    esp_err_t err = i2c_master_transmit_receive(device_handle, &dir_register, 1,data_read,read_size,TIMEOUT_MS);
    if (err != ESP_OK)
    {
        return I2C_ERR_FAIL;
    }

    return I2C_OK; // Retorna I2C_OK si la operación fue exitosa
}





/***** */