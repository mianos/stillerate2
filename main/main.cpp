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

#include "sdkconfig.h"

#include "MqttClient.h"
#include "WifiManager.h"
#include "SettingsManager.h"
#include "Button.h"
#include "PidController.h"
#include "Max31865Sensor.h"

#include "MqttContext.h"
#include "Emulation.h"
#include "PidControlTimer.h"
#include "SensorLoopTask.h"
#include "RESTMotorController.h"
#include "SettingsManager.h"

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

    doc.AddItem("version", 3);
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
	doc.AddItem("period", 0);	// starts with Pid loop stopped
	doc.AddItem("reflux", 0);	// reflux pump at 0
	doc.AddItem("condenser", 0);	// condenser pump at 0
	settings.toJsonWrapper(doc);
	pid.toJsonWrapper(doc);
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



esp_err_t pidSettingsHandler(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context); // Explicit cast required
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "pidSettingsHandler", "Context cannot be nullptr");
    // Use settings and other parameters to handle settings
	ESP_LOGI(TAG, "PID settings handler called with '%s'", data.ToString().c_str());
	ctx->pid->setParametersFromJsonWrapper(data);
	return ESP_OK;
}

esp_err_t pidEmulationHandler(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context); // Explicit cast required
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "pidEmulationHandler", "Context cannot be nullptr");
	auto* emu = ctx->emu;
	emu->fromJsonWrapper(data);
	ESP_LOGI(TAG, "pidemulation called '%s'", emu->toJsonWrapper().ToString().c_str());
	return ESP_OK;
}


esp_err_t pidRun(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context); // Explicit cast required
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "pidRun", "Context cannot be nullptr");
	auto* ptimer  = ctx->ptimer;

    int period;
    if (!data.GetField("period", period, true)) { // mandatory flag so not present will return false
        ESP_LOGE(TAG, "pidRun failed: 'period' not found in JSON");
        return ESP_FAIL; // Handle the error appropriately
    }
    ptimer->start(period * 1000);
	ESP_LOGI(TAG, "pidRun called '%s'", data.ToString().c_str());
	return ESP_OK;
}


esp_err_t reportPidParamsHandler(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context); // Explicit cast required
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "reportParamsHandler", "Context cannot be nullptr");
    JsonWrapper doc;
	doc.AddTime();
	ctx->pid->toJsonWrapper(doc);
    client->publish("tele/" + client->sensorName + "/pidparams", doc.ToString());;
	ESP_LOGI(TAG, "sent pid params '%s'", doc.ToString().c_str());
	return ESP_OK;
}

esp_err_t pumpHandler(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context);
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "pumpHandler", "Context cannot be nullptr");

	std::string name;
    ESP_RETURN_ON_FALSE(data.GetField("name", name, true), ESP_FAIL, "pumpHandler", "pump name field not present");
	float value;
    ESP_RETURN_ON_FALSE(data.GetField("value", value, true), ESP_FAIL, "pumpHandler", "pump value field not present");
	if (name == "condenser") {
		ctx->condenser_pump->setDutyPercentage(value);
	} else if (name == "reflux") {
		ctx->reflux_pump->setDutyPercentage(value);
	} else {
        ESP_LOGE(TAG, "invalid pump name '%s'", name.c_str());
        return ESP_FAIL;
	}
	ESP_LOGI(TAG, "motor handler called '%s'", data.ToString().c_str());
	return ESP_OK;
}

esp_err_t updateSettingsHandler(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context);
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "updateSettingsHandler", "Context cannot be nullptr");
	ESP_LOGI(TAG, "Update Settings handler called with '%s'", data.ToString().c_str());
    ctx->settings->updateFromJsonWrapper(data);
	return ESP_OK;
}

esp_err_t wifiConfigHandler(MqttClient* client, const std::string& topic, const JsonWrapper& data, void* context) {
    auto* ctx = static_cast<MqttContext*>(context);
	ESP_RETURN_ON_FALSE(context, ESP_FAIL, "wifiConfig", "Context cannot be nullptr");

    std::string host_name;
    if (data.GetField("host_name", host_name, true)) {
        ctx->wifiManager->configSetHostName(host_name);
        ESP_LOGI(TAG, "Updated Wi-Fi hostname to '%s'", host_name.c_str());
    } else {
        ESP_LOGW(TAG, "Missing or invalid 'host_name'");
    }
	return ESP_OK;
}

extern "C" void app_main() {
	wifiSemaphore = xSemaphoreCreateBinary();
	NvsStorageManager nv;
	SettingsManager settings(nv);

	//auto trigger = GPIOWrapper(GPIO_NUM_2);
    RESTMotorController reflux_pump(settings.refluxPump, "reflux");
    RESTMotorController condenser_pump(settings.condenserPump, "condenser");
    Max31865Sensor boiler_temp(GPIO_NUM_1);
    Max31865Sensor reflux_temp(GPIO_NUM_2);

	ESP_LOGI(TAG, "Settings %s", settings.toJson().c_str());
	PIDController pid(nv, 78.4, 0.01, 0.7, 0.01, 50, 0);


	esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = settings.mqttBrokerUri.c_str();
    mqtt_cfg.credentials.username = settings.mqttUserName.c_str();
    mqtt_cfg.credentials.client_id = settings.sensorName.c_str();
    mqtt_cfg.credentials.authentication.password = settings.mqttUserPassword.c_str();
    MqttClient client(mqtt_cfg, settings.sensorName);

	Emulation emu;
	PIDControlTimer ptimer(pid, client, settings, reflux_temp, reflux_pump, emu);
	WiFiManager wifiManager(nv, localEventHandler, nullptr);
	xTaskCreate(button_task, "button_task", 2048, &wifiManager, 10, NULL);

	MqttContext ctx{&pid, &emu, &ptimer, &reflux_pump, &condenser_pump, &settings, &wifiManager};

	SensorLoopTask slt{client, settings, boiler_temp};

	std::string topic = "cmnd/" + settings.sensorName + "/pid";
	client.registerHandler(topic, std::regex(topic), pidSettingsHandler, &ctx);
	topic = "cmnd/" + settings.sensorName + "/pidemulation";
	client.registerHandler(topic, std::regex(topic), pidEmulationHandler, &ctx);
	topic = "cmnd/" + settings.sensorName + "/pidrun";
	client.registerHandler(topic, std::regex(topic), pidRun, &ctx);
	topic = "cmnd/" + settings.sensorName + "/pump";
	client.registerHandler(topic, std::regex(topic), pumpHandler, &ctx);
	topic = "cmnd/" + settings.sensorName + "/pidparams";
	client.registerHandler(topic, std::regex(topic), reportPidParamsHandler, &ctx);
	topic = "cmnd/" + settings.sensorName + "/settings";
	client.registerHandler(topic, std::regex(topic), updateSettingsHandler, &ctx);
	topic = "cmnd/" + settings.sensorName + "/wificonfig";
	client.registerHandler(topic, std::regex(topic), wifiConfigHandler, &ctx);


    if (xSemaphoreTake(wifiSemaphore, portMAX_DELAY) ) {
		initialize_sntp(settings);
		client.start();
		PublishMqttInit(client, settings, pid);

		while (true) {
			vTaskDelay(pdMS_TO_TICKS(4000)); 
			//ESP_LOGI(TAG, "val %lu", value);
			// Check whether to increment or decrement
		}
    }
}

