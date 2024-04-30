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

#include "sdkconfig.h"

#include "MqttClient.h"
#include "WifiManager.h"
#include "SettingsManager.h"
#include "Button.h"
#include "Motormanager.h"
#include "PidController.h"
#include "Max31865Sensor.h"

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

struct MqttContext {
	PIDController *pid;
};

void pidSettingsHandler(MqttClient* client, const std::string& topic, cJSON* data, void* context) {
    auto* settings = static_cast<MqttContext*>(context); // Explicit cast required
    // Use settings and other parameters to handle settings
	ESP_LOGI(TAG, "PID settings handler called");
}

extern "C" void app_main() {
	wifiSemaphore = xSemaphoreCreateBinary();
	NvsStorageManager nv;
	SettingsManager settings(nv);

	//auto trigger = GPIOWrapper(GPIO_NUM_2);
	
	// FIX ..
	//E (474) ledc: ledc_timer_del(592): LEDC is not initialized
//	0x42008e1e: MotorController::MotorController(int, ledc_timer_t, ledc_channel_t, ledc_mode_t, int, int, ledc_timer_bit_t) at /Users/rfo/wa/stillerate2/main/Motormanager.h:31 (discriminator 1)
//    MotorController motor1(5, LEDC_TIMER_0, LEDC_CHANNEL_0); // Motor 1 on GPIO 5
//    MotorController motor2(18, LEDC_TIMER_1, LEDC_CHANNEL_1); // Motor 2 on GPIO 18
    // Set initial duty cycle (e.g., 50% duty for motor 1)
   // motor1.setDuty(4096);
    // Set different duty cycle for motor 2 if desired
   // motor2.setDuty(2048);
    Max31865Sensor sensor1(GPIO_NUM_7);
    Max31865Sensor sensor2(GPIO_NUM_6);

	ESP_LOGI(TAG, "Settings %s", settings.toJson().c_str());



	pid_ctrl_parameter_t params = {};
	params.kp = 3.0;
	params.ki = 0.1;
	params.kd = 1.0;
	params.max_output = 160.0;
	params.min_output = 0;
	params.min_integral = -100.0;
	params.max_integral = 100.0;
	params.cal_type = PID_CAL_TYPE_POSITIONAL;
	PIDController pid(nv, params);


	esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = settings.mqttBrokerUri.c_str();
    mqtt_cfg.credentials.username = settings.mqttUserName.c_str();
    mqtt_cfg.credentials.client_id = settings.sensorName.c_str();
    mqtt_cfg.credentials.authentication.password = settings.mqttUserPassword.c_str();
    MqttClient client(mqtt_cfg, settings.sensorName);

	MqttContext ctx{&pid};
	std::string topic = "cmnd/" + settings.sensorName + "/pid";
	client.registerHandler(topic, std::regex(topic), pidSettingsHandler, &ctx);

	WiFiManager wifiManager(nv, localEventHandler, nullptr);
	xTaskCreate(button_task, "button_task", 2048, &wifiManager, 10, NULL);

    if (xSemaphoreTake(wifiSemaphore, portMAX_DELAY) ) {
		initialize_sntp(settings);
		client.start();
		PublishMqttInit(client, settings);

        ESP_LOGI(TAG, "Main task continues after WiFi connection.");

		while (true) {
			float reflux = sensor1.measure();
			float boiler = sensor2.measure();
			if (reflux >= 0 && boiler >= 0) {
				ESP_LOGI(Tag, "T1: %.4f, T2: %.4f, diff: %.4f", reflux, boiler, reflux - boiler);

#if 0
				float output;
				float error = 74.8 - reflux;
				if (!pid.compute(error, output)) {
					ESP_LOGE(TAG, "failed to calculate pid error");
				} else {
					ESP_LOGI(TAG, "error %.4f pid out %.4f", error, output);
				}
#endif
			}
			vTaskDelay(pdMS_TO_TICKS(500)); 
			//ESP_LOGI(TAG, "val %lu", value);
			// Check whether to increment or decrement
		}
    }
}

