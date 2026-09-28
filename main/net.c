
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "esp_mac.h" // esp_base_mac_addr_get
#include "mqtt_client.h"
#include "Sensors.h"

static const char *TAG = "NET";

extern void *sensors;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event */
/* - are we connected to the AP with an IP? */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

esp_err_t wifi_init_sta(void)
{
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
		ESP_EVENT_ANY_ID,
		&event_handler,
		NULL,
		&instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
		IP_EVENT_STA_GOT_IP,
		&event_handler,
		NULL,
		&instance_got_ip));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_ESP_WIFI_SSID,
			.password = CONFIG_ESP_WIFI_PASSWORD,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,

			.pmf_cfg = {
				.capable = true,
				.required = false
			},
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	esp_err_t ret_value = ESP_OK;
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
		WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);

        extern esp_mqtt_client_handle_t mqtt_pub(void);
        esp_mqtt_client_handle_t handle = mqtt_pub();
        ESP_LOGI(TAG, "MQTT HANDLE %p", handle);

        void sntp_main(void);
        sntp_main();

	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
		ret_value = ESP_FAIL;
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
		ret_value = ESP_FAIL;
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);
	return ret_value;
}

esp_err_t query_mdns_host(const char * host_name, char *ip)
{
	ESP_LOGD(__FUNCTION__, "Query A: %s", host_name);

	struct esp_ip4_addr addr;
	addr.addr = 0;

	esp_err_t err = mdns_query_a(host_name, 10000,	&addr);
	if(err){
		if(err == ESP_ERR_NOT_FOUND){
			ESP_LOGW(__FUNCTION__, "%s: Host was not found!", host_name);
		} else {
			ESP_LOGE(__FUNCTION__, "Query Failed: %s", esp_err_to_name(err));
		}
		return ESP_FAIL;
	}

	ESP_LOGD(__FUNCTION__, "Query A: %s.local resolved to: " IPSTR, host_name, IP2STR(&addr));
	sprintf(ip, IPSTR, IP2STR(&addr));
	return ESP_OK;
}

void convert_mdns_host(char * from, char * to)
{
	ESP_LOGI(__FUNCTION__, "from=[%s]",from);
	strcpy(to, from);
	char *sp;
	sp = strstr(from, ".local");
	if (sp == NULL) return;

	int _len = sp - from;
	ESP_LOGD(__FUNCTION__, "_len=%d", _len);
	char _from[128];
	strcpy(_from, from);
	_from[_len] = 0;
	ESP_LOGI(__FUNCTION__, "_from=[%s]", _from);

	char _ip[128];
	esp_err_t ret = query_mdns_host(_from, _ip);
	ESP_LOGI(__FUNCTION__, "query_mdns_host=%d _ip=[%s]", ret, _ip);
	if (ret != ESP_OK) return;

	strcpy(to, _ip);
	ESP_LOGI(__FUNCTION__, "to=[%s]", to);
}

#define CONFIG_MDNS_HOSTNAME "FIXME"
void initialize_mdns(void)
{
	//initialize mDNS
	ESP_ERROR_CHECK( mdns_init() );
	//set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(CONFIG_MDNS_HOSTNAME) );
	ESP_LOGI(TAG, "mdns hostname set to: [%s]", CONFIG_MDNS_HOSTNAME);

#if 0
	//set default mDNS instance name
	ESP_ERROR_CHECK( mdns_instance_name_set("ESP32 with mDNS") );
#endif
}

esp_err_t query_mdns_host(const char * host_name, char *ip);
void convert_mdns_host(char * from, char * to);


#define CONFIG_USE_MQTT 1
#if CONFIG_USE_MQTT


//static const char *TAG = "PUB";

//extern const uint8_t root_cert_pem_start[] asm("_binary_root_cert_pem_start");
//extern const uint8_t root_cert_pem_end[] asm("_binary_root_cert_pem_end");

EventGroupHandle_t mqtt_status_event_group;
#define MQTT_CONNECTED_BIT BIT2

extern MessageBufferHandle_t xMessageBufferTrans;
extern size_t xItemSize;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t)handler_args;
	switch (event->event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
			xEventGroupSetBits(mqtt_status_event_group, MQTT_CONNECTED_BIT);
            Sensors_mqttConnected(sensors, mqtt_client);

            break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
			xEventGroupClearBits(mqtt_status_event_group, MQTT_CONNECTED_BIT);
            Sensors_mqttConnected(sensors, 0);
			break;
		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOG_BUFFER_HEXDUMP(TAG, event->topic, event->topic_len, ESP_LOG_INFO);
            ESP_LOG_BUFFER_HEXDUMP(TAG, event->data, event->data_len, ESP_LOG_INFO);
            Sensors_mqttData(sensors, event->topic, event->topic_len, event->data, event->data_len);
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
			break;
		default:
			ESP_LOGI(TAG, "Other event id:%d", event->event_id);
			break;
	}
	return;
}

esp_mqtt_client_handle_t mqtt_pub()
{
	ESP_LOGI(TAG, "Start Publish Broker:%s", CONFIG_MQTT_BROKER);

	// Create Event Group
	mqtt_status_event_group = xEventGroupCreate();
	configASSERT( mqtt_status_event_group );
	xEventGroupClearBits(mqtt_status_event_group, MQTT_CONNECTED_BIT);

	// Set client id from mac
	uint8_t mac[8];
	ESP_ERROR_CHECK(esp_base_mac_addr_get(mac));
	for(int i=0;i<8;i++) {
		ESP_LOGD(TAG, "mac[%d]=%x", i, mac[i]);
	}
	char client_id[64];
	sprintf(client_id, "pub-%02x%02x%02x%02x%02x%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	ESP_LOGI(TAG, "client_id=[%s]", client_id);

	// Resolve mDNS host name
	char ip[128];
	char uri[138];
	ESP_LOGI(TAG, "CONFIG_MQTT_BROKER=[%s]", CONFIG_MQTT_BROKER);
	convert_mdns_host(CONFIG_MQTT_BROKER, ip);
	ESP_LOGI(TAG, "ip=[%s]", ip);
#if CONFIG_MQTT_TRANSPORT_OVER_TCP
	ESP_LOGI(TAG, "MQTT_TRANSPORT_OVER_TCP");
	sprintf(uri, "mqtt://%.60s:%d", ip, CONFIG_MQTT_PORT_TCP);
#elif CONFIG_MQTT_TRANSPORT_OVER_SSL
	ESP_LOGI(TAG, "MQTT_TRANSPORT_OVER_SSL");
	sprintf(uri, "mqtts://%.60s:%d", ip, CONFIG_MQTT_PORT_SSL);
#elif CONFIG_MQTT_TRANSPORT_OVER_WS
	ESP_LOGI(TAG, "MQTT_TRANSPORT_OVER_WS");
	sprintf(uri, "ws://%.60s:%d/mqtt", ip, CONFIG_MQTT_PORT_WS);
#elif CONFIG_MQTT_TRANSPORT_OVER_WSS
	ESP_LOGI(TAG, "MQTT_TRANSPORT_OVER_WSS");
	sprintf(uri, "wss://%.60s:%d/mqtt", ip, CONFIG_MQTT_PORT_WSS);
#endif
	ESP_LOGI(TAG, "uri=[%s]", uri);

	// Initialize MQTT configuration structure
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = uri,
#if CONFIG_MQTT_TRANSPORT_OVER_TCP
#elif CONFIG_MQTT_TRANSPORT_OVER_SSL
		.broker.verification.certificate = (const char *)root_cert_pem_start,
#elif CONFIG_MQTT_TRANSPORT_OVER_WS
#elif CONFIG_MQTT_TRANSPORT_OVER_WSS
		.broker.verification.certificate = (const char *)root_cert_pem_start,
#endif
#if CONFIG_BROKER_AUTHENTICATION
		.credentials.username = CONFIG_AUTHENTICATION_USERNAME,
		.credentials.authentication.password = CONFIG_AUTHENTICATION_PASSWORD,
#endif
		.credentials.client_id = client_id
	};

#if CONFIG_MQTT_PROTOCOL_V_3_1_1
	ESP_LOGI(TAG, "MQTT_PROTOCOL_V_3_1_1");
	mqtt_cfg.session.protocol_ver = MQTT_PROTOCOL_V_3_1_1;
#elif CONFIG_MQTT_PROTOCOL_V_5
	ESP_LOGI(TAG, "MQTT_PROTOCOL_V_5");
	mqtt_cfg.session.protocol_ver = MQTT_PROTOCOL_V_5;
#endif

	esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
	esp_mqtt_client_start(mqtt_client);
	xEventGroupWaitBits(mqtt_status_event_group, MQTT_CONNECTED_BIT, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "Connected to MQTT Broker");

#if 0
	char buffer[xItemSize];
	while (1) {
		size_t received = xMessageBufferReceive(xMessageBufferTrans, buffer, sizeof(buffer), portMAX_DELAY);
		ESP_LOGI(TAG, "xMessageBufferReceive received=%d", received);
		if (received > 0) {
			ESP_LOGI(TAG, "xMessageBufferReceive buffer=[%.*s]",received, buffer);
			EventBits_t EventBits = xEventGroupGetBits(mqtt_status_event_group);
			ESP_LOGI(TAG, "EventBits=0x%"PRIx32, EventBits);
			if (EventBits & MQTT_CONNECTED_BIT) {
				int msg_id = esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_PUB_TOPIC, buffer, received, 1, 0);
				ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
			} else {
				ESP_LOGW(TAG, "Disconnect to MQTT Broker. Skip to send");
			}
		} else {
			 ESP_LOGE(TAG, "xMessageBufferReceive fail");
			 break;
		}
	} // end while

	// Stop connection
	ESP_LOGI(TAG, "Task Delete");
	esp_mqtt_client_stop(mqtt_client);
	vTaskDelete(NULL);
#endif
return mqtt_client;
}

#endif
