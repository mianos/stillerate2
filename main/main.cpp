#include <cstdlib>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_netif.h"

#include "sdkconfig.h"

#include "MqttClient.h"
#include "WifiManager.h"
#include "SettingsManager.h"
#include "Button.h"
#include "OneWireManager.h"
#include "Motormanager.h"

static const char *TAG = "stillerate2";

static SemaphoreHandle_t wifiSemaphore;

void initialize_sntp(SettingsManager& settings) {
	setenv("TZ", settings.tz.c_str(), 1);
	tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, settings.ntpServer.c_str());
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP service initialized");
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

void PublishMqttInit(MqttClient& client, SettingsManager& settings) {
    JsonWrapper doc;

    doc.AddItem("version", 2);
    doc.AddItem("name", settings.sensorName);
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

    std::string status_topic = std::string("tele/") + settings.sensorName + "/init";
    std::string output = doc.ToString();
    client.publish(status_topic, output);
}



static void localEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
	    xSemaphoreGive(wifiSemaphore);
	}
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

extern "C" void app_main() {
	wifiSemaphore = xSemaphoreCreateBinary();
	NvsStorageManager nv;
	SettingsManager settings(nv);

	auto dsx = OneWireManager();

    MotorController motor1(5, LEDC_TIMER_0, LEDC_CHANNEL_0); // Motor 1 on GPIO 5
    MotorController motor2(18, LEDC_TIMER_1, LEDC_CHANNEL_1); // Motor 2 on GPIO 18
    // Set initial duty cycle (e.g., 50% duty for motor 1)
    motor1.setDuty(4096);
    // Set different duty cycle for motor 2 if desired
    motor2.setDuty(2048);

	ESP_LOGI(TAG, "Settings %s", settings.toJson().c_str());
    MqttClient client(settings);
	WiFiManager wifiManager(nv, localEventHandler, nullptr);

	xTaskCreate(button_task, "button_task", 2048, &wifiManager, 10, NULL);

    if (xSemaphoreTake(wifiSemaphore, portMAX_DELAY) ) {
		initialize_sntp(settings);
		client.start();
		PublishMqttInit(client, settings);
        ESP_LOGI(TAG, "Main task continues after WiFi connection.");
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(100)); 
			//ESP_LOGI(TAG, "val %lu", value);
			// Check whether to increment or decrement
		}
    }
}

