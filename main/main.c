#include <stdio.h> // Funciones estándar de entrada/salida (printf)
#include <string.h> // Funciones para manipulación de cadenas (strstr, strtok)
#include "freertos/FreeRTOS.h" // Núcleo del sistema operativo en tiempo real
#include "freertos/task.h" // Funciones para manejo de tareas y retardos
#include "driver/uart.h" // Controlador de interfaz UART para comunicación serie
#include "driver/gpio.h" // Controlador de GPIO para configuración de pines

// PLACA Y MÓDULOS
// ESP32-WROOM-32 2AC7Z ← A7670SA-FASE (GNSS): 3V3, GND, G16 (RX), G17 (TX)

// CONFIGURACIÓN UART
#define UART_NUM UART_NUM_2 // UART número 2 para comunicación con GPS
#define UART_RX_PIN 16 // Pin GPIO16 para recepción (RX)
#define UART_TX_PIN 17 // Pin GPIO17 para transmisión (TX)
#define UART_BAUD_RATE 9600 // Velocidad de baudios estándar para GPS NMEA (9600 bps)
#define UART_BUF_SIZE 1024 // Tamaño del buffer de recepción UART

// VARIABLES GLOBALES
char buffer_nmea[256]; // Buffer para almacenar sentencias NMEA recibidas
int indice_buffer = 0; // Índice actual en el buffer NMEA

// PROTOTIPOS DE FUNCIONES
static int procesar_sentencia_nmea(const char *sentencia_nmea, float *latitud, float *longitud, float *altitud);
static float convertir_grados_minutos_a_decimal(const char *grados_minutos, char direccion);

// FUNCIONES
// Inicializa la comunicación UART con el módulo GPS
static void uart_inicializar()
{
    uart_config_t config_uart = 
    {
        .baud_rate = UART_BAUD_RATE, // Velocidad de 9600 baudios
        .data_bits = UART_DATA_8_BITS, // 8 bits de datos
        .parity = UART_PARITY_DISABLE, // Sin paridad
        .stop_bits = UART_STOP_BITS_1, // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Sin control de flujo
        .rx_flow_ctrl_thresh = 122, // Umbral de control de flujo RX (no usado)
        .source_clk = UART_SCLK_DEFAULT // Fuente de reloj por defecto
    };
    
    uart_param_config(UART_NUM, &config_uart); // Aplicar parámetros de configuración
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // Configurar pines TX/RX
    uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0); // Instalar controlador UART con buffer
}

// Lee datos del GPS a través de UART y extrae sentencias NMEA
static int leer_datos_gps(float *latitud, float *longitud, float *altitud)
{
    uint8_t datos_recibidos[128]; // Buffer para datos recibidos
    int longitud_datos = uart_read_bytes(UART_NUM, datos_recibidos, sizeof(datos_recibidos) - 1, pdMS_TO_TICKS(100)); // Leer hasta 128 bytes con timeout 100ms
    
    if (longitud_datos > 0) // Si se recibieron datos
    {
        datos_recibidos[longitud_datos] = '\0'; // Terminar cadena con null
        
        for (int indice = 0; indice < longitud_datos; indice++) // Procesar cada byte recibido
        {
            char caracter_actual = datos_recibidos[indice]; // Caracter actual
            
            if (caracter_actual == '\n' || indice_buffer >= sizeof(buffer_nmea) - 1) // Fin de línea o buffer lleno
            {
                buffer_nmea[indice_buffer] = '\0'; // Terminar cadena en buffer
                
                if (strstr(buffer_nmea, "$GPGGA") != NULL) // Buscar sentencia GPGGA (Global Positioning System Fix Data)
                {
                    if (procesar_sentencia_nmea(buffer_nmea, latitud, longitud, altitud)) // Procesar sentencia
                    {
                        indice_buffer = 0; // Resetear índice del buffer
                        return 1; // Éxito: coordenadas obtenidas
                    }
                }
                
                indice_buffer = 0; // Resetear índice del buffer
            }
            else if (caracter_actual == '\r') // Ignorar retorno de carro
            {
                continue; // Saltar al siguiente caracter
            }
            else // Caracter normal
            {
                buffer_nmea[indice_buffer++] = caracter_actual; // Almacenar caracter en buffer
            }
        }
    }
    
    return 0; // Fallo: no se obtuvieron coordenadas
}

// Procesa una sentencia NMEA GPGGA y extrae coordenadas
static int procesar_sentencia_nmea(const char *sentencia_nmea, float *latitud, float *longitud, float *altitud)
{
    char copia_sentencia[256]; // Copia de la sentencia para tokenización
    strncpy(copia_sentencia, sentencia_nmea, sizeof(copia_sentencia) - 1); // Copiar sentencia
    copia_sentencia[sizeof(copia_sentencia) - 1] = '\0'; // Asegurar terminación
    
    char *tokens[20]; // Array para almacenar tokens
    int contador_tokens = 0; // Contador de tokens
    
    char *token_actual = strtok(copia_sentencia, ","); // Tokenizar por comas
    
    while (token_actual != NULL && contador_tokens < 20) // Mientras haya tokens y espacio
    {
        tokens[contador_tokens++] = token_actual; // Almacenar token
        token_actual = strtok(NULL, ","); // Obtener siguiente token
    }
    
    if (contador_tokens >= 10) // GPGGA necesita al menos 10 campos
    {
        // Campo 2: Latitud en formato GGMM.MMMMM
        // Campo 3: Dirección latitud (N/S)
        // Campo 4: Longitud en formato GGGMM.MMMMM
        // Campo 5: Dirección longitud (E/W)
        // Campo 9: Altitud en metros
        
        if (strlen(tokens[2]) > 0 && strlen(tokens[4]) > 0 && strlen(tokens[9]) > 0) // Verificar campos no vacíos
        {
            *latitud = convertir_grados_minutos_a_decimal(tokens[2], tokens[3][0]); // Convertir latitud
            *longitud = convertir_grados_minutos_a_decimal(tokens[4], tokens[5][0]); // Convertir longitud
            *altitud = atof(tokens[9]); // Convertir altitud a float
            
            return 1; // Éxito: coordenadas extraídas
        }
    }
    
    return 0; // Fallo: sentencia inválida o incompleta
}

// Convierte formato NMEA (GGMM.MMMMM) a grados decimales
static float convertir_grados_minutos_a_decimal(const char *grados_minutos, char direccion)
{
    float valor_grados_minutos = atof(grados_minutos); // Convertir cadena a float
    
    int grados_enteros = (int)(valor_grados_minutos / 100); // Extraer grados (primeros 2-3 dígitos)
    float minutos_decimal = valor_grados_minutos - (grados_enteros * 100.0f); // Extraer minutos (resto)
    
    float grados_decimal = grados_enteros + (minutos_decimal / 60.0f); // Convertir a grados decimales
    
    if (direccion == 'S' || direccion == 'W') // Sur u Oeste son valores negativos
    {
        grados_decimal = -grados_decimal;
    }
    
    return grados_decimal; // Retornar grados decimales
}

// PUNTO DE PARTIDA
void app_main()
{
    // INICIALIZACIÓN UART
    uart_inicializar(); // Configurar comunicación UART con GPS
    
    // VARIABLES PARA COORDENADAS
    float latitud_actual = 0.0f;
    float longitud_actual = 0.0f;
    float altitud_actual = 0.0f;
    
    int contador_conexion = 0; // Contador para mensajes "Connecting..."
    
    // BUCLE PRINCIPAL INFINITO
    while (1)
    {
        if (leer_datos_gps(&latitud_actual, &longitud_actual, &altitud_actual)) // Intentar leer coordenadas
        {
            // Mostrar coordenadas obtenidas con formato solicitado
            printf("Coordinates: %.6f (latitude), %.6f (longitude), %.1f (altitude)\n", 
                   latitud_actual, longitud_actual, altitud_actual);
            
            contador_conexion = 0; // Resetear contador al obtener coordenadas
        }
        else // No se obtuvieron coordenadas
        {
            contador_conexion++; // Incrementar contador
            
            if (contador_conexion % 10 == 0) // Mostrar "Connecting..." cada 10 intentos (~10 segundos)
            {
                printf("Coordinates: Connecting...\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo entre lecturas
    }
}