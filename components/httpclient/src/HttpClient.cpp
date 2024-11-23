#include "HttpClient.h"
#include <esp_log.h>
#include <utility>

static const char* TAG = "HttpClient";

// Constructor: Initializes the HTTP client with the provided URL
HttpClient::HttpClient(const std::string& url) : url(url) {
    esp_http_client_config_t config = {};
    config.url = this->url.c_str();
    config.event_handler = HttpClient::handleHttpEvent;
    config.user_data = this; // Pass this instance for callback context
    config.timeout_ms = timeout;

    client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
    }
}

// Destructor: Cleans up the HTTP client handle
HttpClient::~HttpClient() {
    if (client) {
        esp_http_client_cleanup(client);
    }
}

// Move constructor
HttpClient::HttpClient(HttpClient&& other) noexcept
    : client(std::exchange(other.client, nullptr)),
      url(std::move(other.url)),
      timeout(other.timeout),
      responseBuffer(std::move(other.responseBuffer)) {}

// Move assignment operator
HttpClient& HttpClient::operator=(HttpClient&& other) noexcept {
    if (this != &other) {
        if (client) {
            esp_http_client_cleanup(client);
        }

        client = std::exchange(other.client, nullptr);
        url = std::move(other.url);
        timeout = other.timeout;
        responseBuffer = std::move(other.responseBuffer);
    }
    return *this;
}

// Set the timeout for the HTTP client
void HttpClient::setTimeout(int milliseconds) {
    timeout = milliseconds;
    if (client) {
        esp_http_client_set_timeout_ms(client, timeout);
    }
}

// Perform an HTTP POST request
std::pair<bool, std::string> HttpClient::post(const std::string& postData) {
    if (!client) {
        ESP_LOGE(TAG, "HTTP client is not initialized");
        return {false, ""};
    }

    // Clear previous response
    responseBuffer.clear();

    esp_http_client_set_url(client, url.c_str());
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, postData.c_str(), postData.size());

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
        return {false, ""};
    }

    int statusCode = esp_http_client_get_status_code(client);
    if (statusCode != 200) {
        ESP_LOGE(TAG, "HTTP POST returned status code: %d", statusCode);
        return {false, ""};
    }

    return {true, std::string(responseBuffer.begin(), responseBuffer.end())};
}

// Handle HTTP events and populate the response buffer
esp_err_t HttpClient::handleHttpEvent(esp_http_client_event_t* evt) {
    auto* client = static_cast<HttpClient*>(evt->user_data);

    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0 && client) {
                client->responseBuffer.insert(
                    client->responseBuffer.end(),
                    static_cast<const char*>(evt->data),
                    static_cast<const char*>(evt->data) + evt->data_len);
            }
            break;

        default:
            break;
    }

    return ESP_OK;
}

