/* The example of RF69
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#if CONFIG_WIFI_MANAGER
#include "wifi_manager.h"
#endif

#include "Sensors.h"

static const char *TAG = "MAIN";

bool radio_init();
bool radio_rawRead(uint8_t *buf, int max, int16_t *rssi);
esp_err_t wifi_init_sta(void);


#define MSG_BUF_SIZE            27
#define MSG_MAX_SIZE            66

#define QUEUE_LENGTH          10
#define ITEM_SIZE             sizeof(int)

QueueHandle_t radioReceiveQueue;
void *sensors;

void rx_task(void *pvParameter)
{
	ESP_LOGI(TAG, "Start");
	uint8_t rxBuf[MSG_MAX_SIZE];

	while(1) {
        int16_t rssi = 0;
		if (radio_rawRead(rxBuf, MSG_MAX_SIZE, &rssi)) {
            ESP_LOGI(TAG, "rssi=%d", rssi);
            ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, MSG_MAX_SIZE, ESP_LOG_INFO);
            bool status = Sensors_decodeMessage(sensors, rxBuf, MSG_MAX_SIZE, rssi);
            if (status) {
                Sensors_publishWeatherdata(sensors);
            }
		} else {
            int received_pin;
            if (xQueueReceive(radioReceiveQueue, &received_pin, 10/*portMAX_DELAY*/) == pdTRUE) {
                ESP_LOGD(TAG, "Interrupt detected on GPIO %d!", received_pin);
            }
        }
		vTaskDelay(10);
	} // end while

	// never reach here
	vTaskDelete( NULL );
}


void app_main()
{
    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s",
               esp_err_to_name(ret));
    }

	// Initialize NVS
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

    sensors = Sensors_new();

#if CONFIG_WIFI_MANAGER
	wifi_manager_init();
	/* start the wifi manager */
	wifi_manager_start();
#else
	ESP_ERROR_CHECK(wifi_init_sta());
#endif

#if CONFIG_USE_MDNS
	// Initialize mDNS
	initialize_mdns();
#endif

    radioReceiveQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
    if (radioReceiveQueue == NULL) {
        printf("Failed to create queue!\n");
        return;
    }

    // My init
	if (!radio_init()) {
		ESP_LOGE(TAG, "RFM69 radio init failed");
		while (1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "RFM69 radio init OK!");

	xTaskCreate(&rx_task, "RX", 1024*10, NULL, 5, NULL);
}

