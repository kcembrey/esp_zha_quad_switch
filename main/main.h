#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "ha/esp_zigbee_ha_standard.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */

#define HA_ESP_SW01_EP                  1
#define HA_ESP_SW02_EP                  2
#define HA_ESP_SW03_EP                  3
#define HA_ESP_SW04_EP                  4
#define HA_ESP_DOOR_LOCK_EP             1
#define HA_ESP_BATTERY_EP               5

#define SW01_GPIO_IN                    GPIO_NUM_18
#define BATTERY_ADC_GPIO                GPIO_NUM_0

#define SERVO_GPIO_OUT                  GPIO_NUM_20

#define DEBOUNCE_TIME_MS                100 

#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID 0x0021

uint8_t get_in_pin(uint8_t sw_ep);
uint8_t get_pin_state_index(uint8_t sw_ep);
void esp_app_switch_handler(uint8_t sw_ep);
void report_switch_attr(uint8_t ep);







/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x0A""Hoobajoob"
#define MODEL_IDENTIFIER                "\x0B""Test Door"


#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                   \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }
