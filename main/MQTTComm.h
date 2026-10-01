///////////////////////////////////////////////////////////////////////////////////////////////////
// Based on
// mqtt_comm.h
//
// MQTT communication
// Code shared between BresserWeaterSensor<MQTT|MQTTCustom|MQTTWifiMgr>.ino
//
//
// https://github.com/matthias-bs/BresserWeatherSensorReceiver
//
//
// created: 02/2025
//
//
// MIT License
//
// Copyright (c) 2025 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// History:
//
// 20250221 Created from BresserWeatherSensorMQTT.ino
// 20250226 Added parameter 'retain' to publishWeatherdata()
// 20250227 Added publishControlDiscovery()
// 20250420 removed AUTO_DISCOVERY here, as it is defined in sketch
// 20250811 Increased PAYLOAD_SIZE
// 20260403 Added PAYLOAD_EXTRA_SIZE for stack-allocated payloadExtra buffer
// 20260403 Changed sensor_info members from String to const char* to avoid heap allocation
//          in haAutoDiscovery(); changed publishStatusDiscovery/publishControlDiscovery
//          parameters from String to const char*
// 20260510 Added display_name to sensor_info for user-defined HA device names
//
// ToDo:
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#define PAYLOAD_SIZE 400      // maximum MQTT message size
// Worst-case extra payload: wind_dir_txt(20) + wind_gust_bft(18) + wind_avg_bft(17) +
// dewpoint_c(22) + perceived_temp_c(29) + wgbt(15) + JSON overhead(7) = 128 B.
// 160 provides a ~25% safety margin. Increase if new fields are added to jsonExtra.
#define PAYLOAD_EXTRA_SIZE 160 // maximum size for extra (derived) payload

//#include <Arduino.h>
#include <string>
#include <vector>
#include <time.h>
//#include <MQTT.h>
//#include <ArduinoJson.h>
#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"
#include "Sensors.h"
#include "WeatherUtils.h"
#include "RainGauge.h"
#include "Lightning.h"

#include "mqtt_client.h"

class MQTTComm {

    public:
    MQTTComm(WeatherSensor &weatherSensor_) : Hostname("sensor"), weatherSensor(weatherSensor_) {
        
        // Map sensor IDs to Names - replace by your own IDs!

 sensor_map = std::vector<SensorMap> {
    {0x4819, "bresser"},
//    {0x21103427, "WeatherSensor"},
//    {0x67566300, "SoilSensor"},
//    {0x5680, "AirQualitySensor"},
//    {0x28966796, "LeakageSensor"},
//    {0xeefb, "LightningSensor"},
//    {0x22400873, "PoolThermometer"}
    //{0x83750871, "SoilMoisture-1"}
    };

 mqttTopics = {
    .pubStatus = "status",
    .pubRadio = "radio",
    .pubData = "data",
    .pubCombined = "combined",
    .pubRssi = "rssi",
    .pubExtra = "extra",
    .pubInc = "sensors_inc",
    .pubExc = "sensors_exc",
    .subReset = "reset",
    .subGetInc = "get_sensors_inc",
    .subGetExc = "get_sensors_exc",
    .subSetInc = "set_sensors_inc",
    .subSetExc = "set_sensors_exc"
};


}

    // MQTT topics structure for organized access
    struct MQTTTopics {
        const char* pubStatus;
        const char* pubRadio;
        const char* pubData;
        const char* pubCombined;
        const char* pubRssi;
        const char* pubExtra;
        const char* pubInc;
        const char* pubExc;
        const char* subReset;
        const char* subGetInc;
        const char* subGetExc;
        const char* subSetInc;
        const char* subSetExc;
    };

    //extern void mqtt_setup(void);

    // Sensor information for Home Assistant auto discovery
    struct sensor_info
    {
        const char* manufacturer;
        const char* model;
        const char* identifier;
        const char* display_name; // optional: overrides "manufacturer model" as HA device name
    };

    /*!
    * \brief (Re-)Connect to WLAN and connect MQTT broker
    */
    void mqtt_connect(void);


    /*!
        * Convert sensor IDs from JSON string to byte array
        * 
        * \param ids list of sensor IDs
        * \param json JSON string
        * \param buf buffer for storing sensor IDs
        * 
        * \returns size in bytes
        */
    uint8_t convSensorsJson(std::string json, uint8_t *buf);

    /*!
        * Set sensors include list from JSON string
        *
        * \param json JSON string
        */
    void setSensorsIncJson(std::string json);

    /*!
        * Set sensors exclude list from JSON string
        *
        * \param json JSON string
        */
    void setSensorsExcJson(std::string json);
        
    /*!
    * \brief MQTT message received callback
    *
    * \param topic   MQTT topic
    * \param payload MQTT payload
    */
    void messageReceived(const std::string topic, const std::string payload);

    /*!
    * \brief Publish weather data as MQTT message
    *
    * \param complete Indicate that entire data is complete, regardless of the flags temp_ok/wind_ok/rain_ok
    *                 (which reflect only the state of the last message)
    */
    void publishWeatherdata(bool complete = false, bool retain = false);

    /*!
    * \brief Publish radio receiver info as JSON string via MQTT
    *
    * Publish RSSI: Received Signal Strength Indication
    */
    void publishRadio(void);

    /*!
    * \brief Home Assistant Auto-Discovery
    *
    * Create and publish MQTT messages for Home Assistant auto-discovery.
    */
    void haAutoDiscovery(void);

    /*!
    * \brief Publish auto-discovery configuration for Home Assistant
    *
    * \param info          Sensor information (manufacturer, model, identifier, optional display_name)
    * \param sensor_name   Sensor name (e.g. "Outside Temperature")
    * \param sensor_id     Sensor ID (unique)
    * \param device_class  Device class (e.g. temperature, humidity, etc.)
    * \param unit          Unit of measurement
    * \param state_topic   State topic; MQTT topic where sensor data is published
    * \param value_json    Sensor value in MQTT message JSON string
    */
    void publishAutoDiscovery(const struct sensor_info info, const char *sensor_name, const uint32_t sensor_id, const char *device_class, const char *unit, const char *state_topic, const char *value_json);

    /*!
    * \brief Publish Home Assistant auto discovery for MQTT node status
    *
    * \param name  Control name
    * \param topic MQTT topic
    */
    void publishStatusDiscovery(const char* name, const char* topic);

    /*!
    * \brief Publish Home Assistant auto discovery for receiver control
    *
    * \param name  Control name
    * \param topic MQTT topic
    */
    void publishControlDiscovery(const char* name, const char* topic);

    void setClient(esp_mqtt_client_handle_t client_) ;

    void setHostName(std::string hostname) {
        Hostname = hostname;
    };

    void publishStatus(void);

    protected:

    bool sensorName(uint32_t sensor_id, char* buf, size_t buf_size);
    std::string sensorName(uint32_t sensor_id);

    std::string Hostname;
    MQTTTopics mqttTopics;
    esp_mqtt_client_handle_t client = 0;
    RainGauge rainGauge;
    Lightning lightning;
    std::vector<SensorMap> sensor_map;
    WeatherSensor &weatherSensor;
};
#endif // MQTT_COMM_H
