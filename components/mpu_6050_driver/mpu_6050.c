#include "mpu_6050.h"

hal_i2c_bus_handle_t i2c_handle;
hal_i2c_device_handle_t mpu_6050_handle;



#define REG_DATA_SAMPLE_RATE 0x19
#define REG_DATA_CONFIG_BAND 0x1A
#define REG_DATA_INT_PIN_CFG 0x37
#define REG_DATA_INT_ENABLE 0x38
#define REG_DATA_START 0x3B
#define REG_DATA_OFFSET_ACC_START 0x00
#define REG_DATA_OFFSET_ACC_X 0x00
#define REG_DATA_OFFSET_ACC_Y 0x02
#define REG_DATA_OFFSET_ACC_Z 0x04
#define REG_DATA_OFFSET_GYRO 0x08
#define REG_DATA_OFFSET_GYRO_X 0x08
#define REG_DATA_OFFSET_GYRO_Y 0x0A
#define REG_DATA_OFFSET_GYRO_Z 0x0C
#define REG_DATA_OFFSET_TEMP 0x06
#define DIR_ALL_REGISTER_ACC (REG_DATA_START + REG_DATA_OFFSET_ACC)
#define DIR_ALL_REGISTER_GYRO (REG_DATA_START + REG_DATA_OFFSET_GYRO)
#define DIR_ALL_REGISTER_TEMP (REG_DATA_START + REG_DATA_OFFSET_TEMP)

uint8_t register_interrup[8]={REG_DATA_SAMPLE_RATE,0x00,REG_DATA_CONFIG_BAND,0x00,REG_DATA_INT_PIN_CFG,0x00,REG_DATA_INT_ENABLE,0x00};

//inicializa el bus i2c y crea un handle para el dispositivo mpu6050
void bus_init(){
    hal_i2c_data_config_t bus_config = {
        .i2c_port=0,
        .sda_io_num=I2C_MASTER_SDA_IO,
        .scl_io_num=I2C_MASTER_SCL_IO

    };

    hal_i2c_master_init(&bus_config,&i2c_handle);

}


void mpu_6050_init(uint8_t device_address, uint32_t speed, interrupt_config_t *interrup_config){

    hal_i2c_device_bus_master_config_t mpu_config = {
        .device_address=device_address,
        .scl_speed_hz=speed
    };
    hal_i2c_add_device(&mpu_config,&mpu_6050_handle,i2c_handle);
    // Despertar el MPU-6050 (Escribir 0x00 en el registro 0x6B)
    uint8_t wakeup_data[2] = {0x6B, 0x00}; // {Dirección_Registro, Valor}
    mpu_6050_write(wakeup_data, 2);
    
    // Pequeño retardo para permitir que el sensor se estabilice tras despertar

    if (interrup_config!=NULL) {
    register_interrup[1]= interrup_config->sample_rate; // Configuración de la frecuencia de muestreo
    mpu_6050_write(&register_interrup[0],2); // Configuración de la frecuencia de corte del filtro pasa bajo
    register_interrup[3]= interrup_config->bandwidth; // Configuración de la frecuencia de muestreo
    mpu_6050_write(&register_interrup[2],2);
    register_interrup[5]= (interrup_config->int_level)<<7;
    mpu_6050_write(&register_interrup[4],2);
    register_interrup[7]= (interrup_config->Data_rdy)<<0;
    mpu_6050_write(&register_interrup[6],2);
    }
}


void mpu_6050_write(uint8_t *data, size_t data_length){
    hal_i2c_master_write(data,data_length,mpu_6050_handle);
}

void mpu_6050_read_acceleration(axis_t axis, int16_t *buffer_out){
    uint8_t start_register = REG_DATA_START;
    size_t bytes_to_read = 0;
    
    switch(axis){
        case X_AXIS:
            start_register += REG_DATA_OFFSET_ACC_X;
            bytes_to_read = 2;
            break;
        case Y_AXIS:
            start_register += REG_DATA_OFFSET_ACC_Y;
            bytes_to_read = 2;
            break;
        case Z_AXIS:
            start_register += REG_DATA_OFFSET_ACC_Z;
            bytes_to_read = 2;
            break;
        case XY_AXIS:
            start_register += REG_DATA_OFFSET_ACC_X;
            bytes_to_read = 4;
            break;
        case XZ_AXIS:
            start_register += REG_DATA_OFFSET_ACC_X;
            bytes_to_read = 6; // Se leen 6 para poder llegar hasta Z
            break;
        case YZ_AXIS:
            start_register += REG_DATA_OFFSET_ACC_Y;
            bytes_to_read = 4;
            break;
        case ALL_AXIS:
            start_register += REG_DATA_OFFSET_ACC_X;
            bytes_to_read = 6;
            break;
    }
    
    uint8_t data_read[6] = {0}; 
    
    // 2. Ejecutar la lectura I2C
    hal_i2c_master_read_register(start_register, data_read, bytes_to_read, mpu_6050_handle);
    
    // 3. Acomodar los datos en el buffer del usuario
    if (axis == XZ_AXIS) {
        // Guardamos X (High y Low)
        buffer_out[0] = (int16_t)((data_read[0]<<8 )| data_read[1]);
        buffer_out[1] = (int16_t)((data_read[4]<<8 )| data_read[5]);

    } else {
        // Para todos los demás casos, la memoria es contigua
        for (int i = 0; i < bytes_to_read; i=i+2) {
            buffer_out[i/2] = (int16_t)((data_read[i]<<8 )| data_read[i+1]);
        }
    }
}

void mpu_6050_read_gyroscope(axis_t axis, int16_t *buffer_out){
    uint8_t start_register = REG_DATA_START;
    size_t bytes_to_read = 0;
    
    switch(axis){
        case X_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_X;
            bytes_to_read = 2;
            break;
        case Y_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_Y;
            bytes_to_read = 2;
            break;
        case Z_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_Z;
            bytes_to_read = 2;
            break;
        case XY_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_X;
            bytes_to_read = 4;
            break;
        case XZ_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_X;
            bytes_to_read = 6;
            break;
        case YZ_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_Y;
            bytes_to_read = 4;
            break;
        case ALL_AXIS:
            start_register += REG_DATA_OFFSET_GYRO_X;
            bytes_to_read = 6;
            break;
    }
    
    uint8_t data_read[6] = {0}; 
    
    hal_i2c_master_read_register(start_register, data_read, bytes_to_read, mpu_6050_handle);
    
    if (axis == XZ_AXIS) {
        buffer_out[0] = (int16_t)((data_read[0]<<8 )| data_read[1]);
        buffer_out[1] = (int16_t)((data_read[4]<<8 )| data_read[5]);
    } else {
        for (int i = 0; i < bytes_to_read; i=i+2) {
            buffer_out[i/2] = (int16_t)((data_read[i]<<8 )| data_read[i+1]);
        }
    }
}