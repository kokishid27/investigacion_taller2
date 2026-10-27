#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "mpu_6050.h"
#include "driver/gpio.h"
#define PIN_INTERRUPCION GPIO_NUM_4

#define TECNICA 2

QueueHandle_t datosQueue;
int16_t buffer_acc[3] = {0}; // Buffer para almacenar datos de aceleración
int16_t buffer_gyro[3] = {0}; // Buffer para almacenar datos de giroscopio
const bool SIMULAR_CARGA_REAL = false; // Cambiar a false para desactivar la simulación de carga real



#if TECNICA==1

// 
// TAREA 1: ADQUISICIÓN 
void vTaskAdquisicion(void *pvParameters) {

    /* interrupt_config_t int_config = {
        .Data_rdy = 1, // Habilita interrupción de datos listos
        .int_level = 0, // Nivel de interrupción (activo en alto)
        .sample_rate = 9, // Frecuencia de muestreo (100 Hz)
        .bandwidth = BAND_44HZ // Frecuencia de corte del filtro pasa bajo
    };*/
    bus_init(); // Inicializa el bus I2C
    mpu_6050_init(0x68, 400000, NULL); // Inicializa el MPU6050 

    const TickType_t xFrecuencia = pdMS_TO_TICKS(10); // 10 ms = 100 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Variables para las métricas de rendimiento
    uint64_t tiempo_inicio_experimento = esp_timer_get_time();
    uint64_t tiempo_anterior = tiempo_inicio_experimento;
    uint32_t muestras_totales = 0;
    uint64_t suma_delta_t = 0;
    bool experimento_activo = true;

    while (1) {
        uint64_t tiempo_actual = esp_timer_get_time();
        
        if (experimento_activo) {
            // 1. Calcular el Delta T (Tiempo entre la muestra anterior y la actual)
            uint32_t delta_t_us = (uint32_t)(tiempo_actual - tiempo_anterior);
            tiempo_anterior = tiempo_actual;

            // 2. Lectura del sensor
            mpu_6050_read_acceleration(ALL_AXIS, buffer_acc);
            
            // 3. Acumular datos para la media aritmética y conteo
            muestras_totales++;
            suma_delta_t += delta_t_us;
            
            uint64_t tiempo_transcurrido_ms = (tiempo_actual - tiempo_inicio_experimento) / 1000;

           
            printf("%llu,%d,%d,%d,%lu\n", tiempo_transcurrido_ms, buffer_acc[0], buffer_acc[1], buffer_acc[2], delta_t_us);

            // 5. Verificar si ya pasaron 60 segundos (60,000,000 microsegundos)
            if ((tiempo_actual - tiempo_inicio_experimento) >= 60000000ULL) {
                experimento_activo = false; // Detener la captura
                
                // CÁLCULO DE MÉTRICAS FINALES
                
                // Muestreo real (Media Aritmética)
                float promedio_delta_t_us = (float)suma_delta_t / muestras_totales;
                float frecuencia_real_hz = 1000000.0f / promedio_delta_t_us;
                
                // Pérdida de datos
                int muestras_esperadas = 6000; // 60 segundos * 100 Hz
                int muestras_perdidas = muestras_esperadas - muestras_totales;
                if (muestras_perdidas < 0) muestras_perdidas = 0; // Evitar negativos si hay sobremuestreo
                
                float porcentaje_perdida = ((float)muestras_perdidas / muestras_esperadas) * 100.0f;

                // Imprimir el reporte final
                printf("\n=========================================\n");
                printf("      RESULTADOS DEL EXPERIMENTO (60s)   \n");
                printf("=========================================\n");
                printf("1. Muestras Esperadas : %d\n", muestras_esperadas);
                printf("2. Muestras Capturadas: %lu\n", muestras_totales);
                printf("3. Muestras Perdidas  : %d (%.2f%% de perdida)\n", muestras_perdidas, porcentaje_perdida);
                printf("4. Muestreo Real Medio: %.2f us (%.2f Hz)\n", promedio_delta_t_us, frecuencia_real_hz);
                printf("=========================================\n");
                printf("Importa la 5ta columna en MATLAB y usa la funcion std() para obtener el Jitter.\n\n");
            }
        }

        // Dormir hasta el siguiente ciclo estricto de 10ms (100 Hz)
        // ESTO FALTABA EN TU CÓDIGO ORIGINAL PARA RESPETAR LOS 100Hz
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

// ==========================================
// TAREA 2: PROCESAMIENTO (Filtros Digitales)
// ==========================================
void vTaskProcesamiento(void *pvParameters) {
    const TickType_t xFrecuencia = pdMS_TO_TICKS(10); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    volatile float acc_x = 1.0, acc_y = 0.5, acc_z = 9.81;
    volatile float magnitud_filtrada = 0;

    while (1) {
        if (SIMULAR_CARGA_REAL) {
            for (int i = 0; i < 150; i++) {
                magnitud_filtrada = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
                magnitud_filtrada *= sin(i * 0.01f);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

// ==========================================
// TAREA 3: ESTIMACIÓN (Cinemática y E/S)
// ==========================================
void vTaskEstimacion(void *pvParameters) {
    const TickType_t xFrecuencia = pdMS_TO_TICKS(50); // Corre a 20 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    volatile float x_pos = 0.0, y_pos = 0.0, theta = 0.0;
    while (1) {
        if (SIMULAR_CARGA_REAL) {
            for (int i = 0; i < 1000; i++) {
                x_pos += cos(theta) * 0.05f;
                y_pos += sin(theta) * 0.05f;
                theta += 0.01f;
            }
            // En experimento real, aquí se imprimen valores de estimación si se desea
        }
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

// ==========================================
// FUNCIÓN PRINCIPAL
// ==========================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(5000)); // Retardo de 5 segundos para abrir el monitor serial tranquilamente

    // Crear las 3 tareas fijadas al NÚCLEO 1 con PRIORIDAD 5.
    xTaskCreatePinnedToCore(vTaskAdquisicion, "Task_Adq", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskProcesamiento, "Task_Proc", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskEstimacion, "Task_Est", 4096, NULL, 5, NULL, 1);

    vTaskDelay(portMAX_DELAY); 
}
#endif






#if TECNICA==2





// Manejador de la tarea para la notificación
static TaskHandle_t task_adquisicion_handle = NULL;

// Rutina de Servicio de Interrupción (ISR) alojada en RAM
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // 1. Validar que la tarea ya fue creada (el manejador NO es NULL)
    if (task_adquisicion_handle != NULL) {
        BaseType_t TareaMayorPrioridad = pdFALSE;
        
        // 2. Notificar a la tarea solo si es seguro hacerlo
        vTaskNotifyGiveFromISR(task_adquisicion_handle, &TareaMayorPrioridad);
        
        // 3. Forzar el cambio de contexto
        if (TareaMayorPrioridad == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}
// ==========================================
// TAREA 1: ADQUISICIÓN (Técnica de Polling)
// ==========================================


void vTaskAdquisicion(void *pvParameters) {

     bus_init(); // Inicializa el bus 
     interrupt_config_t int_config = {
        .Data_rdy = 1, // Habilita interrupción de datos listos
        .int_level = 0, // Nivel de interrupción (activo en alto)
        .sample_rate = 9, // Frecuencia de muestreo (100 Hz)
        .bandwidth = BAND_44HZ // Frecuencia de corte del filtro pasa bajo
    };
   
    mpu_6050_init(0x68, 400000, &int_config); // Inicializa el MPU6050 

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,       // Interrupción por flanco de subida
        .pin_bit_mask = (1ULL << PIN_INTERRUPCION), // Seleccionar el pin 4
        .mode = GPIO_MODE_INPUT,              // Modo entrada
        .pull_up_en = 0,                      // Desactivar pull-up
        .pull_down_en = 1                     // Activar pull-down (estado bajo por defecto)
    };
    gpio_config(&io_conf);
   // 3. Instalar el servicio de interrupciones global (0 = flags por defecto)
    gpio_install_isr_service(0);
    
    // 4. Adjuntar nuestro manejador específico al pin 4
    gpio_isr_handler_add(PIN_INTERRUPCION, gpio_isr_handler, (void*) PIN_INTERRUPCION);

    
    const TickType_t xFrecuencia = pdMS_TO_TICKS(10); // 10 ms = 100 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Variables para las métricas de rendimiento
    uint64_t tiempo_inicio_experimento = esp_timer_get_time();
    uint64_t tiempo_anterior = tiempo_inicio_experimento;
    uint32_t muestras_totales = 0;
    uint64_t suma_delta_t = 0;
    bool experimento_activo = true;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t tiempo_actual = esp_timer_get_time();
        
        if (experimento_activo) {
            // 1. Calcular el Delta T (Tiempo entre la muestra anterior y la actual)
            uint32_t delta_t_us = (uint32_t)(tiempo_actual - tiempo_anterior);
            tiempo_anterior = tiempo_actual;

            // 2. Lectura del sensor
            mpu_6050_read_acceleration(ALL_AXIS, buffer_acc);
            mpu_6050_read_gyroscope(ALL_AXIS, buffer_gyro); // Si también quieres leer el giroscopio
            
            // 3. Acumular datos para la media aritmética y conteo
            muestras_totales++;
            suma_delta_t += delta_t_us;
            
            uint64_t tiempo_transcurrido_ms = (tiempo_actual - tiempo_inicio_experimento) / 1000;

            // 4. Imprimir formato CSV para MATLAB
            // Formato: Tiempo(ms), AccX, AccY, AccZ, DeltaT(us)
            // MATLAB usará la 5ta columna (DeltaT) para calcular std() y obtener el Jitter
            printf("%llu,%d,%d,%d,%lu\n", tiempo_transcurrido_ms, buffer_acc[0], buffer_acc[1], buffer_acc[2], delta_t_us);

            // 5. Verificar si ya pasaron 60 segundos (60,000,000 microsegundos)
            if ((tiempo_actual - tiempo_inicio_experimento) >= 60000000ULL) {
                experimento_activo = false; // Detener la captura
                
                // --- CÁLCULO DE MÉTRICAS FINALES ---
                
                // Muestreo real (Media Aritmética)
                float promedio_delta_t_us = (float)suma_delta_t / muestras_totales;
                float frecuencia_real_hz = 1000000.0f / promedio_delta_t_us;
                
                // Pérdida de datos
                int muestras_esperadas = 6000; // 60 segundos * 100 Hz
                int muestras_perdidas = muestras_esperadas - muestras_totales;
                if (muestras_perdidas < 0) muestras_perdidas = 0; // Evitar negativos si hay sobremuestreo
                
                float porcentaje_perdida = ((float)muestras_perdidas / muestras_esperadas) * 100.0f;

                // Imprimir el reporte final
                printf("\n=========================================\n");
                printf("      RESULTADOS DEL EXPERIMENTO (60s)   \n");
                printf("=========================================\n");
                printf("1. Muestras Esperadas : %d\n", muestras_esperadas);
                printf("2. Muestras Capturadas: %lu\n", muestras_totales);
                printf("3. Muestras Perdidas  : %d (%.2f%% de perdida)\n", muestras_perdidas, porcentaje_perdida);
                printf("4. Muestreo Real Medio: %.2f us (%.2f Hz)\n", promedio_delta_t_us, frecuencia_real_hz);
                printf("=========================================\n");
                printf("Importa la 5ta columna en MATLAB y usa la funcion std() para obtener el Jitter.\n\n");
            }
        }
    }
}

// ==========================================
// TAREA 2: PROCESAMIENTO (Filtros Digitales)
// ==========================================
void vTaskProcesamiento(void *pvParameters) {
    const TickType_t xFrecuencia = pdMS_TO_TICKS(10); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    volatile float acc_x = 1.0, acc_y = 0.5, acc_z = 9.81;
    volatile float magnitud_filtrada = 0;

    while (1) {
        if (SIMULAR_CARGA_REAL) {
            for (int i = 0; i < 150; i++) {
                magnitud_filtrada = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
                magnitud_filtrada *= sin(i * 0.01f);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

// ==========================================
// TAREA 3: ESTIMACIÓN (Cinemática y E/S)
// ==========================================
void vTaskEstimacion(void *pvParameters) {
    const TickType_t xFrecuencia = pdMS_TO_TICKS(50); // Corre a 20 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    volatile float x_pos = 0.0, y_pos = 0.0, theta = 0.0;
    while (1) {
        if (SIMULAR_CARGA_REAL) {
            for (int i = 0; i < 1000; i++) {
                x_pos += cos(theta) * 0.05f;
                y_pos += sin(theta) * 0.05f;
                theta += 0.01f;
            }
            // En experimento real, aquí se imprimen valores de estimación si se desea
        }
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

// ==========================================
// FUNCIÓN PRINCIPAL
// ==========================================
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(5000)); // Retardo de 5 segundos para abrir el monitor serial tranquilamente

    // Crear las 3 tareas fijadas al NÚCLEO 1 con PRIORIDAD 5.
    xTaskCreatePinnedToCore(vTaskAdquisicion, "Task_Adq", 4096, NULL, 6, &task_adquisicion_handle, 1);
    xTaskCreatePinnedToCore(vTaskProcesamiento, "Task_Proc", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskEstimacion, "Task_Est", 4096, NULL, 5, NULL, 1);

    vTaskDelay(portMAX_DELAY); 
}
#endif

#if TECNICA==3


static TaskHandle_t task_adquisicion_handle = NULL;
static TaskHandle_t task_procesamiento_handle = NULL;

static int16_t buffer_acc[3];
static int16_t buffer_gyro[3];

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    if (task_adquisicion_handle != NULL) {
        BaseType_t TareaMayorPrioridad = pdFALSE;
        vTaskNotifyGiveFromISR(task_adquisicion_handle, &TareaMayorPrioridad);
        if (TareaMayorPrioridad == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

void vTaskAdquisicion(void *pvParameters) {
    bus_init(); 
    interrupt_config_t int_config = {
        .Data_rdy = 1, 
        .int_level = 0, 
        .sample_rate = 9, 
        .bandwidth = BAND_44HZ 
    };
   
    mpu_6050_init(0x68, 400000, &int_config); 

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,       
        .pin_bit_mask = (1ULL << PIN_INTERRUPCION), 
        .mode = GPIO_MODE_INPUT,              
        .pull_up_en = 0,                      
        .pull_down_en = 1                     
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_INTERRUPCION, gpio_isr_handler, (void*) PIN_INTERRUPCION);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        mpu_6050_read_acceleration(ALL_AXIS, buffer_acc);
        mpu_6050_read_gyroscope(ALL_AXIS, buffer_gyro); 
        
        if (task_procesamiento_handle != NULL) {
            xTaskNotifyGive(task_procesamiento_handle);
        }
    }
}

void vTaskProcesamiento(void *pvParameters) {
    float acc_x_filtrada = 0.0f;
    int16_t ventana[5] = {0, 0, 0, 0, 0};
    
    uint64_t tiempo_inicio_experimento = esp_timer_get_time();
    uint32_t muestras_totales = 0;
    uint64_t suma_tiempo_procesamiento = 0;
    bool experimento_activo = true;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t tiempo_actual = esp_timer_get_time();

        if (experimento_activo) {
            uint64_t t_inicio_filtro = esp_timer_get_time();
            
            ventana[4] = ventana[3];
            ventana[3] = ventana[2];
            ventana[2] = ventana[1];
            ventana[1] = ventana[0];
            ventana[0] = buffer_acc[0];
            
            acc_x_filtrada = (-3.0f * ventana[4] + 12.0f * ventana[3] + 17.0f * ventana[2] + 12.0f * ventana[1] - 3.0f * ventana[0]) / 35.0f;
            
            uint64_t t_fin_filtro = esp_timer_get_time();
            uint32_t delta_t_proc = (uint32_t)(t_fin_filtro - t_inicio_filtro);
            
            suma_tiempo_procesamiento += delta_t_proc;
            muestras_totales++;

            uint64_t tiempo_transcurrido_ms = (tiempo_actual - tiempo_inicio_experimento) / 1000;

            printf("%llu,%d,%.2f,%lu\n", tiempo_transcurrido_ms, buffer_acc[0], acc_x_filtrada, delta_t_proc);

            if ((tiempo_actual - tiempo_inicio_experimento) >= 60000000ULL) {
                experimento_activo = false; 
                
                float tiempo_medio_proc = (float)suma_tiempo_procesamiento / muestras_totales;

              
                printf("Muestras Procesadas: %lu\n", muestras_totales);
                printf("Tiempo medio de ejecucion por muestra: %.2f us\n", tiempo_medio_proc);
                printf("Exportar a MATLAB para metricas restantes:\n");
                printf("Col 1: Tiempo (ms)\n");
                printf("Col 2: Senal Cruda (Raw)\n");
                printf("Col 3: Senal Filtrada (Savitzky-Golay N=5)\n");
                printf("Col 4: Costo Computacional por muestra (us)\n");
                printf("-> Reduccion de varianza: 100 * (1 - var(Col3)/var(Col2))\n");
                printf("-> Atenuacion freq: FFT(Col3) vs FFT(Col2)\n");
                printf("-> Retardo de grupo: Analisis de fase cruzada Col3 vs Col2\n\n");
            }
        }
    }
}

void vTaskEstimacion(void *pvParameters) {
    const TickType_t xFrecuencia = pdMS_TO_TICKS(50); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    volatile float x_pos = 0.0, y_pos = 0.0, theta = 0.0;
    while (1) {
        if (SIMULAR_CARGA_REAL) {
            for (int i = 0; i < 1000; i++) {
                x_pos += cos(theta) * 0.05f;
                y_pos += sin(theta) * 0.05f;
                theta += 0.01f;
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(5000)); 

    xTaskCreatePinnedToCore(vTaskProcesamiento, "Task_Proc", 4096, NULL, 5, &task_procesamiento_handle, 1);
    xTaskCreatePinnedToCore(vTaskAdquisicion, "Task_Adq", 4096, NULL, 6, &task_adquisicion_handle, 1);
    xTaskCreatePinnedToCore(vTaskEstimacion, "Task_Est", 4096, NULL, 5, NULL, 1);

    vTaskDelay(portMAX_DELAY); 
}
#endif

#if TECNICA==4


static TaskHandle_t task_adquisicion_handle = NULL;
static TaskHandle_t task_procesamiento_handle = NULL;

static int16_t buffer_acc[3];
static int16_t buffer_gyro[3];

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    if (task_adquisicion_handle != NULL) {
        BaseType_t TareaMayorPrioridad = pdFALSE;
        vTaskNotifyGiveFromISR(task_adquisicion_handle, &TareaMayorPrioridad);
        if (TareaMayorPrioridad == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

void vTaskAdquisicion(void *pvParameters) {
    bus_init(); 
    interrupt_config_t int_config = {
        .Data_rdy = 1, 
        .int_level = 0, 
        .sample_rate = 9, 
        .bandwidth = BAND_44HZ 
    };
   
    mpu_6050_init(0x68, 400000, &int_config); 

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,       
        .pin_bit_mask = (1ULL << PIN_INTERRUPCION), 
        .mode = GPIO_MODE_INPUT,              
        .pull_up_en = 0,                      
        .pull_down_en = 1                     
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_INTERRUPCION, gpio_isr_handler, (void*) PIN_INTERRUPCION);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        mpu_6050_read_acceleration(ALL_AXIS, buffer_acc);
        mpu_6050_read_gyroscope(ALL_AXIS, buffer_gyro); 
        
        if (task_procesamiento_handle != NULL) {
            xTaskNotifyGive(task_procesamiento_handle);
        }
    }
}

void vTaskProcesamiento(void *pvParameters) {
    float x_n1 = 0.0f, x_n2 = 0.0f;
    float y_n1 = 0.0f, y_n2 = 0.0f;
    
    const float b0 = 0.067455f;
    const float b1 = 0.134910f;
    const float b2 = 0.067455f;
    const float a1 = -1.142980f;
    const float a2 = 0.412800f;

    float x_n = 0.0f;
    float acc_x_filtrada = 0.0f;

    uint64_t tiempo_inicio_experimento = esp_timer_get_time();
    uint32_t muestras_totales = 0;
    uint64_t suma_tiempo_procesamiento = 0;
    bool experimento_activo = true;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t tiempo_actual = esp_timer_get_time();

        if (experimento_activo) {
            uint64_t t_inicio_filtro = esp_timer_get_time();
            
            x_n = (float)buffer_acc[0];
            
            acc_x_filtrada = (b0 * x_n) + (b1 * x_n1) + (b2 * x_n2) - (a1 * y_n1) - (a2 * y_n2);
            
            x_n2 = x_n1;
            x_n1 = x_n;
            y_n2 = y_n1;
            y_n1 = acc_x_filtrada;
            
            uint64_t t_fin_filtro = esp_timer_get_time();
            uint32_t delta_t_proc = (uint32_t)(t_fin_filtro - t_inicio_filtro);
            
            suma_tiempo_procesamiento += delta_t_proc;
            muestras_totales++;

            uint64_t tiempo_transcurrido_ms = (tiempo_actual - tiempo_inicio_experimento) / 1000;

            printf("%llu,%d,%.2f,%lu\n", tiempo_transcurrido_ms, buffer_acc[0], acc_x_filtrada, delta_t_proc);

            if ((tiempo_actual - tiempo_inicio_experimento) >= 60000000ULL) {
                experimento_activo = false; 
                
                float tiempo_medio_proc = (float)suma_tiempo_procesamiento / muestras_totales;

                printf("\n=========================================\n");
                printf("RESULTADOS ETAPA DE PROCESAMIENTO (60s)\n");
                printf("=========================================\n");
                printf("Muestras Procesadas: %lu\n", muestras_totales);
                printf("Tiempo medio de ejecucion por muestra: %.2f us\n", tiempo_medio_proc);
                printf("=========================================\n");
                printf("Exportar a MATLAB para metricas restantes:\n");
                printf("Col 1: Tiempo (ms)\n");
                printf("Col 2: Senal Cruda (Raw)\n");
                printf("Col 3: Senal Filtrada (Butterworth 2do Orden)\n");
                printf("Col 4: Costo Computacional por muestra (us)\n");
                printf("-> Reduccion de varianza: 100 * (1 - var(Col3)/var(Col2))\n");
                printf("-> Atenuacion freq: FFT(Col3) vs FFT(Col2)\n");
                printf("-> Retardo de grupo: Analisis de fase cruzada Col3 vs Col2\n\n");
            }
        }
    }
}

void vTaskEstimacion(void *pvParameters) {
    const TickType_t xFrecuencia = pdMS_TO_TICKS(50); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    volatile float x_pos = 0.0, y_pos = 0.0, theta = 0.0;
    while (1) {
        if (SIMULAR_CARGA_REAL) {
            for (int i = 0; i < 1000; i++) {
                x_pos += cos(theta) * 0.05f;
                y_pos += sin(theta) * 0.05f;
                theta += 0.01f;
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(5000)); 

    xTaskCreatePinnedToCore(vTaskProcesamiento, "Task_Proc", 4096, NULL, 5, &task_procesamiento_handle, 1);
    xTaskCreatePinnedToCore(vTaskAdquisicion, "Task_Adq", 4096, NULL, 6, &task_adquisicion_handle, 1);
    xTaskCreatePinnedToCore(vTaskEstimacion, "Task_Est", 4096, NULL, 5, NULL, 1);

    vTaskDelay(portMAX_DELAY); 
}
#endif