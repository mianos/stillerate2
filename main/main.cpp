#include <cstdlib>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "esp_check.h"
#include "esp_timer.h" // Added for high-res timing
#include "esp_app_desc.h"

#include "sdkconfig.h"

#include "MqttClient.h"
#include "WifiManager.h"
#include "SettingsManager.h"
#include "Button.h"
#include "PidController.h"
#include "Max31865Sensor.h"

#include "StillContext.h"
#include "Emulation.h"
#include "PidControlTimer.h"
#include "SensorLoopTask.h"
#include "RESTMotorController.h"
#include "SettingsManager.h"
#include "esp_ota_ops.h"
#include "StillWebServer.h"

static const char *TAG = "stillerate2";

static SemaphoreHandle_t wifiSemaphore;

void initialize_sntp(SettingsManager& settings) {
    setenv("TZ", settings.tz.c_str(), 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, settings.ntpServer.c_str());
    esp_sntp_init();
    int max_retry = 200;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && max_retry--) {
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    if (max_retry <= 0) {
        ESP_LOGE(TAG, "Failed to synchronize NTP time");
        return; // Exit if unable to sync
    }
    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    ESP_LOGI("TimeTest", "Current local time and date: %d-%d-%d %d:%d:%d",
             1900 + timeinfo.tm_year, 1 + timeinfo.tm_mon, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void PublishMqttInit(MqttClient& client, SettingsManager& settings, PIDController &pid) {
    JsonWrapper doc;

    doc.AddItem("version", 6);
    const esp_app_desc_t* app = esp_app_get_description();
    doc.AddItem("fw_version", std::string(app->version));
    doc.AddItem("idf_version", std::string(app->idf_ver));
    doc.AddItem("build_date", std::string(app->date) + " " + app->time);
    doc.AddTime();
    doc.AddTime(false, "gmt");

    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE("NET_INFO", "Network interface for STA not found");
        return;
    }
    // Get hostname
    const char* hostname;
    esp_netif_get_hostname(netif, &hostname);
    doc.AddItem("hostname", std::string(hostname));

    // Get IP Address
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        char ip_str[16]; // Buffer to hold the IP address string
        esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
        doc.AddItem("ip", std::string(ip_str));
    } else {
        ESP_LOGE("NET_INFO", "Failed to get IP information");
    }
    doc.AddItem("settings", "cmnd/" + settings.sensorName + "/settings");
    doc.AddItem("sample_time", 0);  // starts with Pid loop stopped
    doc.AddItem("reflux", 0);   // reflux pump at 0
    doc.AddItem("condenser", 0);    // condenser pump at 0
    settings.toJsonWrapper(doc, /*maskPassword=*/true);
    pid.toJsonWrapper(doc);
    std::string status_topic = std::string("tele/") + settings.sensorName + "/init";
    std::string output = doc.ToString();
    client.publish(status_topic, output);
}



// --- OTA rollback verification ---
//
// A freshly-OTA'd image boots in PENDING_VERIFY: the bootloader rolls it back
// to the previous slot on the next reset unless the running app declares itself
// good. We declare it good only once the device is back on the network, so an
// image that boots but can't reach Wi-Fi is rolled back instead of stranding an
// unreachable device. A wired first-flash is UNDEFINED, so this never touches
// normal/bench boots.
constexpr int OTA_VERIFY_TIMEOUT_MS = 120000;
static SemaphoreHandle_t s_got_ip = nullptr;

static void localEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
        xSemaphoreGive(wifiSemaphore);
        xSemaphoreGive(s_got_ip);
    }
}

static void otaVerifyTask(void*) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "OTA: image pending verify; waiting up to %ds for connectivity",
                 OTA_VERIFY_TIMEOUT_MS / 1000);
        if (xSemaphoreTake(s_got_ip, pdMS_TO_TICKS(OTA_VERIFY_TIMEOUT_MS)) == pdTRUE) {
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA: connectivity confirmed, image marked valid");
        } else {
            ESP_LOGE(TAG, "OTA: no IP within timeout; rolling back to previous image");
            esp_ota_mark_app_invalid_rollback_and_reboot();  // reboots on success
            ESP_LOGE(TAG, "OTA: rollback not possible; keeping current image");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    vTaskDelete(nullptr);
}

void button_task(void *pvParameters) {
    WiFiManager *wifiManager = static_cast<WiFiManager*>(pvParameters);  // Cast the void pointer back to WiFiManager pointer

    Button button(static_cast<gpio_num_t>(CONFIG_BUTTON_PIN));
    while (1) {
        if (button.longPressed()) {
            ESP_LOGI("BUTTON", "Long press detected, resetting WiFi settings.");
            wifiManager->clear();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100 ms
    }
}


class GPIOWrapper {
public:
    // Constructor to initialize the GPIO pin as output and set it to a default level
    GPIOWrapper(gpio_num_t pin, int defaultLevel = 0) : pin_(pin) {
        gpio_config_t io_conf{};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << pin_);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        // Set the initial level of the GPIO pin
        gpio_set_level(pin_, defaultLevel);
    }

    // Method to set the GPIO level
    void setLevel(int level) {
        gpio_set_level(pin_, level);
    }

private:
    gpio_num_t pin_; // GPIO pin number
};



extern "C" void app_main() {
    wifiSemaphore = xSemaphoreCreateBinary();
    s_got_ip = xSemaphoreCreateBinary();
    NvsStorageManager nv;
    SettingsManager settings(nv);

    //auto trigger = GPIOWrapper(GPIO_NUM_2);
    RESTMotorController reflux_pump(settings.refluxPump, "reflux");
    RESTMotorController condenser_pump(settings.condenserPump, "condenser");
    Max31865Sensor head_temp(GPIO_NUM_1, "head");
    Max31865Sensor middle_temp(GPIO_NUM_2, "middle");

    ESP_LOGI(TAG, "Settings %s", settings.toJson().c_str());
    PIDController pid(nv);


    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = settings.mqttBrokerUri.c_str();
    mqtt_cfg.credentials.username = settings.mqttUserName.c_str();
    mqtt_cfg.credentials.client_id = settings.sensorName.c_str();
    mqtt_cfg.credentials.authentication.password = settings.mqttUserPassword.c_str();
    mqtt_cfg.session.keepalive = 30;  // broker publishes the Offline will ~1.5x this after a drop
    MqttClient client(mqtt_cfg, settings.sensorName);

    Emulation emu;
    PIDControlTimer ptimer(pid, client, settings, middle_temp, reflux_pump, head_temp, emu);
    WiFiManager wifiManager(nv, localEventHandler, nullptr);
    xTaskCreate(button_task, "button_task", 2048, &wifiManager, 10, NULL);

    xTaskCreate(otaVerifyTask, "ota_verify", 4096, nullptr, 4, nullptr);

    StillContext ctx{&pid, &emu, &ptimer, &reflux_pump, &condenser_pump, &settings, &wifiManager};
    SensorLoopTask slt{client, settings, {{"middle", &middle_temp}, {"head", &head_temp}}};

    if (xSemaphoreTake(wifiSemaphore, portMAX_DELAY) ) {
        initialize_sntp(settings);
        client.start();
        PublishMqttInit(client, settings, pid);

        static WebContext webctx(&wifiManager);
        static StillWebServer web(&webctx, &ctx, &middle_temp, &head_temp);
        web.start();

        // Variables for Pump Health Checking
        int64_t last_pump_check = 0;
        const int64_t check_interval_sec = 30; // Check every 30 seconds

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000)); 
            
            // Get current uptime in seconds
            int64_t now = esp_timer_get_time() / 1000000;

            if (now - last_pump_check > check_interval_sec) {
                last_pump_check = now;

                bool reflux_ok = reflux_pump.isHealthy();
                bool condenser_ok = condenser_pump.isHealthy();

                // Create JSON status
                JsonWrapper pumpStatus;
                pumpStatus.AddItem("reflux_ok", reflux_ok);
                pumpStatus.AddItem("condenser_ok", condenser_ok);

                // Publish to MQTT
                std::string pump_topic = "tele/" + settings.sensorName + "/pumps";
                client.publish(pump_topic, pumpStatus.ToString());
                
                // Optional Logging
                if (!reflux_ok || !condenser_ok) {
                    ESP_LOGW(TAG, "Pump Health Alert! Reflux: %d, Condenser: %d", reflux_ok, condenser_ok);
                }
            }
        }
    }
}
