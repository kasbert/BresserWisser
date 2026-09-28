
#ifndef Sensors_h
#define Sensors_h

#ifdef __cplusplus
#include "WeatherSensor.h"
#endif

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif
    void * Sensors_new();
    void Sensors_delete(void * s);
    bool Sensors_decodeMessage(void * s, const uint8_t *msg, uint8_t msgSize, int16_t rssi);
    void Sensors_mqttConnected(void * s, esp_mqtt_client_handle_t handle);
    void Sensors_mqttData(void * s, const char *topic, int topic_len, const char *data, int data_len);
    bool Sensors_publishWeatherdata(void * s);
#ifdef __cplusplus
}
#endif

#endif
