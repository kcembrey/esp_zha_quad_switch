#include "main.h"

#include "esp_check.h"
#include "string.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "ESP_ZB_QUAD_SWITCH";

bool button01_pressed = false;
bool button02_pressed = false;
bool button03_pressed = false;
bool button04_pressed = false;

void IRAM_ATTR button_isr_handler(void *arg)
{
    static uint32_t last_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    if (current_time - last_isr_time > DEBOUNCE_TIME_MS)
    {
        if ((int)arg == SW01_GPIO_IN)
        {
            button01_pressed = true;
        }
        else if ((int)arg == SW02_GPIO_IN)
        {
            button02_pressed = true;
        }
        else if ((int)arg == SW03_GPIO_IN)
        {
            button03_pressed = true;
        }
        else if ((int)arg == SW04_GPIO_IN)
        {
            button04_pressed = true;
        }
        last_isr_time = current_time;
    }
}

void esp_app_switch_handler(uint8_t sw_ep)
{

    esp_zb_zcl_attr_t *switch_attr = esp_zb_zcl_get_attribute(
        sw_ep,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
    bool cur_val = *(bool *)switch_attr->data_p;
    cur_val = !cur_val;

    uint8_t out_pin = get_out_pin(sw_ep);
    gpio_set_level(out_pin, cur_val);


    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(sw_ep,
                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &cur_val, false);
    esp_zb_lock_release();
}

void report_switch_attr(uint8_t ep)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ep;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
}


static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    ESP_LOGI(TAG, "Started");
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool switch_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    if (
        (message->info.dst_endpoint == HA_ESP_SW01_EP) ||
        (message->info.dst_endpoint == HA_ESP_SW02_EP) ||
        (message->info.dst_endpoint == HA_ESP_SW03_EP) ||
        (message->info.dst_endpoint == HA_ESP_SW04_EP))
    {
        uint8_t sw_ep = message->info.dst_endpoint;
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                switch_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : switch_state;
                ESP_LOGI(TAG, "Switch %d, set to %s", sw_ep, switch_state ? "On" : "Off");
                uint8_t out_pin = get_out_pin(sw_ep);
                gpio_set_level(out_pin, switch_state);
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Received Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {

            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *add_basic_and_identify_clusters_create(
    esp_zb_identify_cluster_cfg_t *identify_cluster_cfg,
    esp_zb_basic_cluster_cfg_t *basic_cluster_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(basic_cluster_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(identify_cluster_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    return cluster_list;
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    };

    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE};

    // On Off End points
    for (uint8_t sw_ep = HA_ESP_SW01_EP; sw_ep < HA_ESP_SW04_EP + 1; sw_ep++)
    {
        uint16_t sw_cur_val = 0x00;
        esp_zb_on_off_cluster_cfg_t sw_in_cfg = {
            .on_off = sw_cur_val};
        esp_zb_attribute_list_t *sw_in_cluster = esp_zb_on_off_cluster_create(&sw_in_cfg);

        esp_zb_cluster_list_t *sw_cluster_list = esp_zb_zcl_cluster_list_create();

        if (sw_ep == HA_ESP_SW01_EP) {
            sw_cluster_list = add_basic_and_identify_clusters_create(
            &identify_cluster_cfg,
            &basic_cluster_cfg);
        }
       

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(sw_cluster_list, sw_in_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

        esp_zb_endpoint_config_t sw_endpoint_config = {
            .endpoint = sw_ep,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
            .app_device_version = 0};
        esp_zb_ep_list_add_ep(ep_list, sw_cluster_list, sw_endpoint_config);
    }
    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_main_loop_iteration();
}

uint8_t get_in_pin(uint8_t sw_ep)
{
    if (sw_ep == HA_ESP_SW01_EP)
    {
        return SW01_GPIO_IN;
    }
    if (sw_ep == HA_ESP_SW02_EP)
    {
        return SW02_GPIO_IN;
    }
    if (sw_ep == HA_ESP_SW03_EP)
    {
        return SW03_GPIO_IN;
    }
    return SW04_GPIO_IN;
}

uint8_t get_out_pin(uint8_t sw_ep)
{
    if (sw_ep == HA_ESP_SW01_EP)
    {
        return SW01_GPIO_OUT;
    }
    if (sw_ep == HA_ESP_SW02_EP)
    {
        return SW02_GPIO_OUT;
    }
    if (sw_ep == HA_ESP_SW03_EP)
    {
        return SW03_GPIO_OUT;
    }
    return SW04_GPIO_OUT;
}


void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Configure GPIOs
    gpio_config_t io_conf_out = {};
    io_conf_out.mode = GPIO_MODE_OUTPUT;
    io_conf_out.pin_bit_mask = (1ULL << SW01_GPIO_OUT) | (1ULL << SW02_GPIO_OUT) | (1ULL << SW03_GPIO_OUT) | (1ULL << SW04_GPIO_OUT);
    gpio_config(&io_conf_out);

    // Ensure all off
    gpio_set_level(SW01_GPIO_OUT, 0);
    gpio_set_level(SW02_GPIO_OUT, 0);
    gpio_set_level(SW03_GPIO_OUT, 0);
    gpio_set_level(SW04_GPIO_OUT, 0);

    gpio_config_t io_conf_in = {};
    io_conf_in.intr_type = GPIO_INTR_POSEDGE;
    io_conf_in.mode = GPIO_MODE_INPUT;
    io_conf_in.pin_bit_mask = (1ULL << SW01_GPIO_IN) | (1ULL << SW02_GPIO_IN) | (1ULL << SW03_GPIO_IN) | (1ULL << SW04_GPIO_IN);
    io_conf_in.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf_in);

    // Install ISR service and attach handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SW01_GPIO_IN, button_isr_handler, (void *)SW01_GPIO_IN);
    gpio_isr_handler_add(SW02_GPIO_IN, button_isr_handler, (void *)SW02_GPIO_IN);
    gpio_isr_handler_add(SW03_GPIO_IN, button_isr_handler, (void *)SW03_GPIO_IN);
    gpio_isr_handler_add(SW04_GPIO_IN, button_isr_handler, (void *)SW04_GPIO_IN);

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    while (true) {
        if (button01_pressed) {
            ESP_LOGI(TAG, "Button 1 Pressed");
            button01_pressed = false;
            esp_app_switch_handler(HA_ESP_SW01_EP);
        }
        if (button02_pressed) {
            ESP_LOGI(TAG, "Button 2 Pressed");
            button02_pressed = false;
            esp_app_switch_handler(HA_ESP_SW02_EP);
        }
        if (button03_pressed) {
            ESP_LOGI(TAG, "Button 3 Pressed");
            button03_pressed = false;
            esp_app_switch_handler(HA_ESP_SW03_EP);
        }
        if (button04_pressed) {
            ESP_LOGI(TAG, "Button 4 Pressed");
            button04_pressed = false;
            esp_app_switch_handler(HA_ESP_SW04_EP);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}