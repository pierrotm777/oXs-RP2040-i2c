
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
//#include "pico/util/queue.h"
#include "tools.h"
#include "config.h"
#include "param.h"
#include "vario.h"
#include "rpm.h"
#include "gps.h"
#include "tools.h"
#include <math.h>

#include "xbus.h"
#include <cstdlib>
#include <cstring>

xbus_sensor_t *sensor;
xbus_sensor_formatted_t *sensor_formatted;

//void i2c_multi_init(PIO pio, uint pin);
//static void i2c_request_handler(uint8_t address);
//void xbus_i2c_handler(uint8_t address);
void i2c_xbus_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
static void set_config();
static uint8_t bcd8(float value, uint8_t precision);
static uint16_t bcd16(float value, uint8_t precision);
static uint32_t bcd32(float value, uint8_t precision);

// void xbus_i2c_handler(uint8_t address)
// {
//     i2c_request_handler(address);
// }


extern CONFIG config;
//queue_t xbuxRxQueue ;

// Globals
double prevSpekVoltage = 0.0;
// XBUS_UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, 
//                                 NO_DATA, NO_DATA,
//                                 NO_DATA, NO_DATA, 
//                                 NO_DATA, NO_DATA, 
//                                 NO_DATA, NO_DATA, 
//                                 NO_DATA, NO_DATA, 
//                                 NO_DATA, NO_DATA, 
//                                 NO_DATA, NO_DATA 
//                                 };

extern uint8_t debugTlm;
extern field fields[];  // list of all telemetry fields that are measured
uint32_t nowSpekMs=millisRp();

uint8_t buffer[64] = {0};
char str_out[64];
uint8_t pin;

void setupXbusSpektrum(){
 
    if ( config.pinPrimIn == 255 || config.pinTlm == 255 || config.protocol != 'X') return; // skip if pins are not defined and if protocol is not Spektrum Xbus  
    
    //sensor = malloc(sizeof(xbus_sensor_t));
    *sensor = (xbus_sensor_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};
    //sensor_formatted = malloc(sizeof(xbus_sensor_formatted_t));
    *sensor_formatted = (xbus_sensor_formatted_t){NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
/*
    stdio_init_all();

    PIO pio = pio0;
    pin = config.pinTlm;//SDA0
    i2c_multi_init(pio, pin);
    i2c_multi_enable_address(XBUS_GPS_LOC_ID);
    i2c_multi_enable_address(XBUS_GPS_STAT_ID);
    i2c_multi_set_request_handler(i2c_request_handler);

    set_config();
    free(sensor);//Deallocate the block of memory, making it available again for future allocations
*/

    gpio_set_function(config.pinPrimIn, GPIO_FUNC_I2C);
	gpio_pull_up(config.pinPrimIn);
    gpio_set_function(config.pinTlm, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinTlm);

    i2c_slave_init(i2c0, XBUS_GPS_LOC_ID, &i2c_xbus_handler);
    //set_config();

}


void i2c_xbus_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) 
    {
        case I2C_SLAVE_RECEIVE: // master has written some data
            //not used here
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            runXbusRequest();
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            //not used here
            break;
        default:
            break;
    }

}

//void xbus_format_sensor(uint8_t address)
void handleXbus(uint8_t address)
{
    static float alt = 0;
    switch (address)
    {
//     case XBUS_AIRSPEED_ID:
//     {
//         sensor_formatted->airspeed->airspeed = __builtin_bswap16(*sensor->airspeed[XBUS_AIRSPEED_AIRSPEED]);
//         if (__builtin_bswap16(sensor_formatted->airspeed->airspeed) > __builtin_bswap16(sensor_formatted->airspeed->max_airspeed))
//             sensor_formatted->airspeed->max_airspeed = sensor_formatted->airspeed->airspeed;
//         break;
//     }
    case XBUS_GPS_LOC_ID:
    {
        uint8_t gps_flags = 0;
        float lat = fields[LATITUDE].value;//*sensor->gps_loc[XBUS_GPS_LOC_LATITUDE];
        if (lat < 0) // N=1,+, S=0,-
            lat *= -1;
        else
            gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT;
        sensor_formatted->gps_loc->latitude = bcd32((uint16_t)(lat / 60) * 100 + fmod(lat, 60), 4);
        float lon = fields[LATITUDE].value;//*sensor->gps_loc[XBUS_GPS_LOC_LONGITUDE];
        if (lon < 0) // E=1,+, W=0,-
            lon *= -1;
        else
            gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_EAST_BIT;
        if (lon >= 6000)
        {
            gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
            lon -= 6000;
        }
        sensor_formatted->gps_loc->longitude = bcd32((uint16_t)(lon / 60) * 100 + fmod(lon, 60), 4);
        sensor_formatted->gps_loc->course = bcd16(fields[GPS_CUMUL_DIST].value,1);//bcd16(*sensor->gps_loc[XBUS_GPS_LOC_COURSE], 1);
        sensor_formatted->gps_loc->hdop = bcd8(fields[GPS_PDOP].value,1);//bcd8(*sensor->gps_loc[XBUS_GPS_LOC_HDOP], 1);
        alt = fields[ALTITUDE].value;//*sensor->gps_loc[XBUS_GPS_LOC_ALTITUDE];
        if (alt < 0)
        {
            gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_NEGATIVE_ALT_BIT;
            alt *= -1;
        }
        sensor_formatted->gps_loc->gps_flags = gps_flags;
        sensor_formatted->gps_loc->altitude_low = bcd16(fmod(alt, 1000), 1);
        break;
    }
    case XBUS_GPS_STAT_ID:
    {
        sensor_formatted->gps_stat->speed = bcd16(*sensor->gps_stat[XBUS_GPS_STAT_SPEED], 1);
        sensor_formatted->gps_stat->utc = bcd32(*sensor->gps_stat[XBUS_GPS_STAT_TIME], 2);
        sensor_formatted->gps_stat->num_sats = bcd8(*sensor->gps_stat[XBUS_GPS_STAT_SATS], 0);
        sensor_formatted->gps_stat->altitude_high = bcd8((uint8_t)(alt / 1000), 0);
        break;
    }
//     case XBUS_ESC_ID:
//     {
//         if (sensor->esc[XBUS_ESC_RPM])
//             sensor_formatted->esc->rpm = __builtin_bswap16(*sensor->esc[XBUS_ESC_RPM] / 10);
//         if (sensor->esc[XBUS_ESC_VOLTAGE])
//             sensor_formatted->esc->volts_input = __builtin_bswap16(*sensor->esc[XBUS_ESC_VOLTAGE] * 100);
//         if (sensor->esc[XBUS_ESC_TEMPERATURE_FET])
//             sensor_formatted->esc->temp_fet = __builtin_bswap16(*sensor->esc[XBUS_ESC_TEMPERATURE_FET] * 10);
//         if (sensor->esc[XBUS_ESC_CURRENT])
//             sensor_formatted->esc->current_motor = __builtin_bswap16(*sensor->esc[XBUS_ESC_CURRENT] * 100);
//         if (sensor->esc[XBUS_ESC_TEMPERATURE_BEC])
//             sensor_formatted->esc->temp_bec = __builtin_bswap16(*sensor->esc[XBUS_ESC_TEMPERATURE_BEC] * 10);
//         break;
//     }
//     case XBUS_BATTERY_ID:
//     {
//         sensor_formatted->battery->current_a = __builtin_bswap16(*sensor->esc[XBUS_BATTERY_CURRENT1] * 10);
//         break;
//     }
//     case XBUS_VARIO_ID:
//     {
//         sensor_formatted->vario->altitude = __builtin_bswap16(*sensor->vario[XBUS_VARIO_ALTITUDE] * 10);
//         sensor_formatted->vario->delta_1000ms = __builtin_bswap16(*sensor->vario[XBUS_VARIO_VSPEED]);
//         break;
//     }
//     case XBUS_RPMVOLTTEMP_ID:
//     {
//         if (sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT])
//             sensor_formatted->rpm_volt_temp->volts = __builtin_bswap16(*sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] * 100);
//         if (sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP])
//             sensor_formatted->rpm_volt_temp->temperature = __builtin_bswap16(*sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP]);
//         break;
//     }
    }
}// end xbus_format_sensor

// void i2c_stop_handler(uint8_t length)
// {
//     //sprintf(str_out, "\nTotal bytes: %u", length);
//     //Serial.print(str_out);
//     printf("Total bytes: %u \n", str_out) ;
// }

static void runXbusRequest()
{
    uint8_t buffer[64];
    if (!fields[NUMSAT].available) return;
    handleXbus(XBUS_GPS_LOC_ID);
    //i2c_write_raw_blocking(i2c0,buffer,sizeof(buffer));// à valider
     i2c_write_blocking(i2c0, XBUS_GPS_LOC_ID, buffer, sizeof(buffer), false); 
}
/*
static void i2c_request_handler(uint8_t address)
{
    // printf("\nAddress: %X, request...", address);
    uint8_t buffer[64];

    switch (address)
    {
    case XBUS_AIRSPEED_ID:
    {
    //     if (!sensor->is_enabled[XBUS_AIRSPEED])
    //         break;
    //     xbus_format_sensor(address);
    //     i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->airspeed);
    //     // vTaskResume(led_task_handle);
    //     // if (debug)
    //     //     {printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
    //     // uint8_t buffer[sizeof(xbus_airspeed_t)];
    //     // memcpy(buffer, sensor_formatted->airspeed, sizeof(xbus_airspeed_t));
    //     // for (int i = 0; i < sizeof(xbus_airspeed_t); i++)
    //     // {
    //     //     printf("%X ", buffer[i]);
    //     // }}
    //     break;
    // }
    // case XBUS_ALTIMETER_ID:
    // {
    //     if (!sensor->is_enabled[XBUS_ALTIMETER])
    //         break;
    //     break;
    // }
    case XBUS_GPS_LOC_ID:
    {
        if (!fields[NUMSAT].available)
            break;
        xbus_format_sensor(address);
        //i2c_multi_set_write_buffer(buffer);
        // vTaskResume(led_task_handle);
        // if (debug)
        //    { printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
        // uint8_t buffer[sizeof(xbus_gps_loc_t)];
        // memcpy(buffer, sensor_formatted->gps_loc, sizeof(xbus_gps_loc_t));
        // for (int i = 0; i < sizeof(xbus_gps_loc_t); i++)
        // {
        //     printf("%X ", buffer[i]);
        // }}
        break;
    }
    case XBUS_GPS_STAT_ID:
    {
        if (!fields[NUMSAT].available)
            break;
        xbus_format_sensor(address);
        //i2c_multi_set_write_buffer(buffer);
        // vTaskResume(led_task_handle);
        // if (debug)
        // {
        //     printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
        //     uint8_t buffer[sizeof(xbus_gps_stat_t)];
        //     memcpy(buffer, sensor_formatted->gps_stat, sizeof(xbus_gps_stat_t));
        //     for (int i = 0; i < sizeof(xbus_gps_stat_t); i++)
        //     {
        //         printf("%X ", buffer[i]);
        //     }
        // }
        break;
    }
//     case XBUS_ESC_ID:
//     {
//         if (!sensor->is_enabled[XBUS_ESC])
//             break;
//         xbus_format_sensor(address);
//         i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->esc);
//         // vTaskResume(led_task_handle);
//         // if (debug)
//         //    { printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
//         // uint8_t buffer[sizeof(xbus_esc_t)];
//         // memcpy(buffer, sensor_formatted->esc, sizeof(xbus_esc_t));
//         // for (int i = 0; i < sizeof(xbus_esc_t); i++)
//         // {
//         //     printf("%X ", buffer[i]);
//         // }}
//         break;
//     }
//     case XBUS_BATTERY_ID:
//     {
//         if (!sensor->is_enabled[XBUS_BATTERY])
//             break;
//         xbus_format_sensor(address);
//         i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->battery);
//         // vTaskResume(led_task_handle);
//         // if (debug)
//         // {
//         //     printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
//         //     uint8_t buffer[sizeof(xbus_battery_t)];
//         //     memcpy(buffer, sensor_formatted->battery, sizeof(xbus_battery_t));
//         //     for (int i = 0; i < sizeof(xbus_battery_t); i++)
//         //     {
//         //         printf("%X ", buffer[i]);
//         //     }
//         // }
//         break;
//     }
//     case XBUS_VARIO_ID:
//     {
//         if (!sensor->is_enabled[XBUS_VARIO])
//             break;
//         xbus_format_sensor(address);
//         i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->vario);
//         // vTaskResume(led_task_handle);
//         // if (debug)
//         //     {printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
//         // uint8_t buffer[sizeof(xbus_vario_t)];
//         // memcpy(buffer, sensor_formatted->vario, sizeof(xbus_vario_t));
//         // for (int i = 0; i < sizeof(xbus_vario_t); i++)
//         // {
//         //     printf("%X ", buffer[i]);
//         // }}
//         break;
//     }
//     case XBUS_RPMVOLTTEMP_ID:
//     {
//         if (!sensor->is_enabled[XBUS_RPMVOLTTEMP])
//             break;
//         xbus_format_sensor(address);
//         i2c_multi_set_write_buffer((uint8_t *)sensor_formatted->rpm_volt_temp);
//         // vTaskResume(led_task_handle);
//         // if (debug)
//         //   {  printf("\nXBUS (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
//         // uint8_t buffer[sizeof(xbus_rpm_volt_temp_t)];
//         // memcpy(buffer, sensor_formatted->rpm_volt_temp, sizeof(xbus_rpm_volt_temp_t));
//         // for (int i = 0; i < sizeof(xbus_rpm_volt_temp_t); i++)
//         // {
//         //     printf("%X ", buffer[i]);
//         // }}
//         break;
        }
    }
}// end i2c_request_handler
*/

static void set_config()
{
    //config_t *config = config_read();
    //TaskHandle_t task_handle;
/*
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        // xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        // xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        // xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        // uart1_notify_task_handle = task_handle;
        // xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW4)
    {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier, config->enable_pwm_out,
                                          config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature, config->esc_hw4_divisor, config->esc_hw4_ampgain, config->esc_hw4_current_thresold, config->esc_hw4_current_max,
                                          malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        // xTaskCreate(esc_hw4_task, "esc_hw4_task", STACK_ESC_HW4, (void *)&parameter, 2, &task_handle);
        // uart1_notify_task_handle = task_handle;
        // xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // if (config->enable_pwm_out)
        // {
        //     xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
        //     pwm_out_task_handle = task_handle;
        //     xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        //     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // }
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor->esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        //i2c_multi_enable_address(XBUS_ESC_ID);
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        //xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        //xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_KONTRONIK)
    {
        esc_kontronik_parameters_t parameter = {config->rpm_multiplier,
                                                config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                                malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        //xTaskCreate(esc_kontronik_task, "esc_kontronik_task", STACK_ESC_KONTRONIK, (void *)&parameter, 2, &task_handle);
        //uart1_notify_task_handle = task_handle;
        //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor->esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_F)
    {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier,
                                            config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                            malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        //xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        //uart1_notify_task_handle = task_handle;
        //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_HV)
    {
        esc_apd_hv_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        //xTaskCreate(esc_apd_hv_task, "esc_apd_hv_task", STACK_ESC_APD_HV, (void *)&parameter, 2, &task_handle);
        //uart1_notify_task_handle = task_handle;
        //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor->esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor->esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor->esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor->is_enabled[XBUS_ESC] = true;
        sensor_formatted->esc = malloc(sizeof(xbus_esc_t));
        *sensor_formatted->esc = (xbus_esc_t){XBUS_ESC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        i2c_multi_enable_address(XBUS_ESC_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
*/
    if (fields[NUMSAT].available)
    {
        // nmea_parameters_t parameter = {config->gps_baudrate,
        //                                malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
        //                                malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        //xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        //uart_pio_notify_task_handle = task_handle;

        sensor->gps_loc[XBUS_GPS_LOC_ALTITUDE] = 0;
        sensor->gps_loc[XBUS_GPS_LOC_LATITUDE] = 0;
        sensor->gps_loc[XBUS_GPS_LOC_LONGITUDE] = 0;
        sensor->gps_loc[XBUS_GPS_LOC_COURSE] = 0;
        sensor->gps_loc[XBUS_GPS_LOC_HDOP] = 0;
        sensor->gps_stat[XBUS_GPS_STAT_SPEED] = 0;
        sensor->gps_stat[XBUS_GPS_STAT_TIME] = 0;
        sensor->gps_stat[XBUS_GPS_STAT_SATS] = 0;
        sensor->gps_stat[XBUS_GPS_STAT_ALTITUDE] = 0;
        sensor->is_enabled[XBUS_GPS_LOC] = true;
        sensor->is_enabled[XBUS_GPS_STAT] = true;
        //sensor_formatted->gps_loc = calloc(1,16);
        *sensor_formatted->gps_loc = (xbus_gps_loc_t){XBUS_GPS_LOC_ID, 0, 0, 0, 0, 0, 0, 0};
        //sensor_formatted->gps_stat = malloc(sizeof(xbus_gps_stat_t));
        *sensor_formatted->gps_stat = (xbus_gps_stat_t){XBUS_GPS_STAT_ID, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_GPS_LOC_ID);
        // i2c_multi_enable_address(XBUS_GPS_STAT_ID);

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    // if (config->enable_analog_voltage)
    // {
    //     voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
    //     //xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] = parameter.voltage;
    //     sensor->is_enabled[XBUS_RPMVOLTTEMP] = true;
    //     sensor_formatted->rpm_volt_temp = malloc(sizeof(xbus_rpm_volt_temp_t));
    //     *sensor_formatted->rpm_volt_temp = (xbus_rpm_volt_temp_t){XBUS_RPMVOLTTEMP_ID, 0, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);

    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->enable_analog_current)
    // {
    //     current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
    //     //xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
    //     sensor->is_enabled[XBUS_BATTERY] = true;
    //     sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
    //     *sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_BATTERY_ID);

    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->enable_analog_ntc)
    // {
    //     ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
    //     //xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP] = parameter.ntc;
    //     sensor->is_enabled[XBUS_RPMVOLTTEMP] = true;
    //     sensor_formatted->rpm_volt_temp = malloc(sizeof(xbus_rpm_volt_temp_t));
    //     *sensor_formatted->rpm_volt_temp = (xbus_rpm_volt_temp_t){XBUS_RPMVOLTTEMP_ID, 0, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->enable_analog_airspeed)
    // {
    //     airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
    //     //xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->airspeed[XBUS_AIRSPEED_AIRSPEED] = parameter.airspeed;
    //     sensor->is_enabled[XBUS_AIRSPEED] = true;
    //     sensor_formatted->airspeed = malloc(sizeof(xbus_airspeed_t));
    //     *sensor_formatted->airspeed = (xbus_airspeed_t){XBUS_AIRSPEED_ID, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_AIRSPEED_ID);

    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->i2c_module == I2C_BMP280)
    // {
    //     bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
    //     //xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
    //     // sensor->vario[XBUS_VARIO_SPEED] = parameter.speed;
    //     sensor->is_enabled[XBUS_VARIO] = true;
    //     sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
    //     *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_VARIO_ID);

    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->i2c_module == I2C_MS5611)
    // {
    //     ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
    //     //xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
    //     sensor->vario[XBUS_VARIO_VSPEED] = parameter.vspeed;
    //     sensor->is_enabled[XBUS_VARIO] = true;
    //     sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
    //     *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_VARIO_ID);

    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->i2c_module == I2C_BMP180)
    // {
    //     bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
    //     //xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
    //     //xQueueSendToBack(tasks_queue_handle, task_handle, 0);

    //     sensor->vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
    //     sensor->vario[XBUS_VARIO_VSPEED] = parameter.vspeed;
    //     sensor->is_enabled[XBUS_VARIO] = true;
    //     sensor_formatted->vario = malloc(sizeof(xbus_vario_t));
    //     *sensor_formatted->vario = (xbus_vario_t){XBUS_VARIO_ID, 0, 0, 0, 0, 0, 0, 0, 0};
    //     i2c_multi_enable_address(XBUS_VARIO_ID);

    //     //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // }
    // if (config->xbus_clock_stretch)
    // {
    //     gpio_set_dir(CLOCK_STRETCH_GPIO, true);
    //     gpio_put(CLOCK_STRETCH_GPIO, false);
    // }
}

uint8_t bcd8(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint8_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%02i", (uint8_t)value);
    for (int i = 0; i < 2; i++)
        output |= (buf[i] - 48) << ((1 - i) * 4);
    return output;
}

uint16_t bcd16(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint16_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%04i", (uint16_t)value);
    for (int i = 0; i < 4; i++)
        output |= (uint16_t)(buf[i] - 48) << ((3 - i) * 4);
    return output;
}

uint32_t bcd32(float value, uint8_t precision)
{
    char buf[10] = {0};
    uint32_t output = 0;
    for (int i = 0; i < precision; i++)
        value = value * 10;
    sprintf(buf, "%08li", (uint32_t)value);
    for (int i = 0; i < 8; i++)
        output |= (uint32_t)(buf[i] - 48) << ((7 - i) * 4);
    return output;
}