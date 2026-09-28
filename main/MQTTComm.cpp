///////////////////////////////////////////////////////////////////////////////////////////////////
// Based on
// mqtt_comm.cpp
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
// Copyright (c) 2026 Matthias Prinke
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
// 20250227 Added publishControlDiscovery()
// 20250228 Added publishStatusDiscovery(), fixed sensorName()
// 20250420 Added timestamp to measurement data, fixed base-topic in extra data
// 20250728 Added combined (Weather & Soil Sensor) MQTT payload
// 20250801 Added Lightning Sensor to combined MQTT payload
// 20250802 Refactored publishWeatherdata() to use ArduinoJson
// 20260113 Fixed HA auto-discovery for UV Index, Light Lux, Wind Direction, 
//          Wind Direction (Cardinal) and Wind Average/Gust Speed (Beaufort)
//          Changed JSON keys from light_klx/ws_light_klx to light_lx/ws_light_lx
//          (values are in Lux)
// 20260221 Refactored publishStatusDiscovery() and publishControlDiscovery()
//          to use JsonDocument and snprintf instead of raw string concatenation
//          Refactored MQTT topic building in publishWeatherdata() to use snprintf
//          instead of String concatenation to reduce temporary object allocations
//          Changed global MQTT topic strings from String to const char* to reduce
//          persistent heap memory usage (~600-750 bytes savings)
//          Refactored MQTT topic declarations using MQTTTopics struct for cleaner
//          organization and maintainability
// 20260312 Fixed latent bug: DATA_TIMESTAMP block used undefined 'sensorData' instead of 'jsonSensor'
//          Removed unused/shadowed outer 'String topic' in haAutoDiscovery()
//          Removed redundant payload.clear() in publishRadio() (local JsonDocument auto-destroyed)
// 20260403 Replaced String payloadSensor/Extra/Combined with stack-allocated char[] to eliminate
//          heap fragmentation from repeated substring() copies on every publish cycle
// 20260403 Issue 9: Eliminated all String heap allocations in haAutoDiscovery() and helper
//          functions (publishAutoDiscovery, publishStatusDiscovery, publishControlDiscovery).
//          Replaced String topic/rssi with stack char[]+snprintf, changed sensor_info members
//          to const char*, updated function signatures to const char* where applicable.
// 20260510 Fixed HA device identifier collision: multiple sensors of the same type now each
//          get a unique identifier derived from sensor_id.
//          Added display_name to sensor_info: HA device name now uses sensor_map name when set.
//
// ToDo:
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <cJSON.h>
#include <time.h>
#include "MQTTComm.h"

#define JSON_ADD_NUMBER(obj, name, value) \
    if (cJSON_AddNumberToObject(obj, name, (double)value) == NULL) { \
        log_e("Error adding number %s", name); \
    }
#define JSON_ADD_STRING(obj, name, value) \
    if (cJSON_AddStringToObject(obj, name, value) == NULL) { \
        log_e("Error adding string %s", name) \
    }


static int publish(esp_mqtt_client_handle_t client, const char *topic, cJSON *json, int qos, int retain, const char *name) {
    char *data = cJSON_PrintUnformatted(json);
    int res = -1;
    if (!data) {
        log_e("Failed to serialize %s : %s", name, topic);
        return -1;
    }
    log_i("%s : %s: %s", name, topic, data);
    if (client) {
        res = esp_mqtt_client_publish(client, topic, data, 0, qos, retain);
    }
    free(data);
    return res;
}

static int publish(esp_mqtt_client_handle_t client, std::string topic, cJSON *json, int qos, int retain, const char *name) {
    char *data = cJSON_PrintUnformatted(json);
    int res = -1;
    if (!data) {
        log_e("Failed to serialize %s : %s", name, topic.c_str());
        return -1;
    }
    log_i("%s : %s: %s", name, topic.c_str(), data);
    if (client) {
        res = esp_mqtt_client_publish(client, topic.c_str(), data, 0, qos, retain);
    }
    free(data);
    return res;
}

static int publish(esp_mqtt_client_handle_t client, const char *topic, const char *data, int len, int qos, int retain) {
    log_i("%s: %s", topic, data);
    if (client) {
        return esp_mqtt_client_publish(client, topic, data, len, qos, retain);
    }
    return -1;
}


void MQTTComm::setClient(esp_mqtt_client_handle_t client_) {
    client = client_;
    if (client) {
        esp_mqtt_client_subscribe(client, (Hostname + "/" + mqttTopics.subReset).c_str(), 0);
        esp_mqtt_client_subscribe(client, (Hostname + "/" + mqttTopics.subGetInc).c_str(), 0);
        esp_mqtt_client_subscribe(client, (Hostname + "/" + mqttTopics.subGetExc).c_str(), 0);
        esp_mqtt_client_subscribe(client, (Hostname + "/" + mqttTopics.subSetInc).c_str(), 0);
        esp_mqtt_client_subscribe(client, (Hostname + "/" + mqttTopics.subSetExc).c_str(), 0);
        log_i("%s: %s\n", (Hostname + "/" + mqttTopics.pubStatus).c_str(), "online");
        publish(client, (Hostname + "/" + mqttTopics.pubStatus).c_str(), "online", 0, 0, false);
    }
};

std::string MQTTComm::sensorName(uint32_t sensor_id)
{
    for (size_t n = 0; n < sensor_map.size(); n++)
    {
        if (sensor_map[n].id == sensor_id)
        {
            return sensor_map[n].name;
        }
    }
    char buf[11];
    snprintf(buf, 10, "%x", (unsigned)sensor_id);
    return std::string(buf);
}

bool MQTTComm::sensorName(uint32_t sensor_id, char* buf, size_t buf_size)
{
    snprintf(buf, buf_size, "%x", (unsigned)sensor_id);
    for (size_t n = 0; n < sensor_map.size(); n++)
    {
        if (sensor_map[n].id == sensor_id)
        {
            snprintf(buf, buf_size, "%s", sensor_map[n].name.c_str());
            return true;
        }
    }
    return false;
}

        /*!
         * Get sensors include/exclude list as JSON string
         *
         * \param ids list of sensor IDs
         * 
         * \returns JSON string
         */
        cJSON *  getSensorsJson(const std::vector<uint32_t> &ids);

        /*!
         * Get sensors include list as JSON string
         *
         * \returns JSON string
         */
        cJSON *  getSensorsIncJson(void);

        /*!
         * Get sensors exclude list as JSON string
         *
         * \returns JSON string
         */
        cJSON *  getSensorsExcJson(void);




// Get sensors include/exclude list as JSON string
cJSON * getSensorsJson(const std::vector<uint32_t> &ids)
{
    cJSON * doc = cJSON_CreateObject();
    cJSON * data = cJSON_AddArrayToObject(doc, "ids");
    for (size_t i = 0; i < ids.size(); i++)
    {
        char buf[12];
        snprintf(buf,12, "0x%08lx", ids[i]);
        // Add to JSON array
        cJSON *str = cJSON_CreateString(buf);
        cJSON_AddItemToArray(data, str);
    }
    return doc;
}

// Get sensors include list as JSON string
cJSON * getSensorsIncJson(WeatherSensor &weatherSensor)
{
    return getSensorsJson(weatherSensor.getSensorIdsInc());
}

// Get sensors exclude list as JSON string
cJSON * getSensorsExcJson(WeatherSensor &weatherSensor)
{
    return getSensorsJson(weatherSensor.getSensorIDsExc());
}

// Convert JSON string to sensor IDs as byte array
uint8_t MQTTComm::convSensorsJson(std::string json, uint8_t *buf)
{
    cJSON * doc   = cJSON_Parse(json.c_str());
    // FIXME FIXME check
    cJSON * data = cJSON_GetObjectItem(doc, "ids");
    for (size_t i = 0; (i < cJSON_GetArraySize(data)) && (i < MAX_SENSOR_IDS); i++)
    {
        cJSON * str2 = cJSON_GetArrayItem(data, i);
        char *str = cJSON_GetStringValue(str2);
        log_d("ID: %s", str);
        for (size_t j=2; j < 10; j += 2) {
            char hexStr[3];
            hexStr[0] = str[j];
            hexStr[1] = str[j+1];
            hexStr[2] = 0;
            *buf++ = (uint8_t)strtol(hexStr, NULL, 16);
        }
    }
    cJSON_Delete(doc);
    return cJSON_GetArraySize(data) * 4;
}

// Set sensors include list from JSON string
void MQTTComm::setSensorsIncJson(std::string  json)
{
    uint8_t buf[MAX_SENSOR_IDS * 4];
    uint8_t size = convSensorsJson(json, buf);
    weatherSensor.setSensorsInc(buf, size);
}

// Set sensors exclude list from JSON string
void MQTTComm::setSensorsExcJson(std::string  json)
{
    uint8_t buf[MAX_SENSOR_IDS * 4];
    uint8_t size = convSensorsJson(json, buf);
    weatherSensor.setSensorsExc(buf, size);
}


// MQTT message received callback
void MQTTComm::messageReceived(const std::string topic_, const std::string payload)
{
    std::string topic = topic_.substr(Hostname.length()+1);

    if (topic == mqttTopics.subReset)
    {
        uint8_t flags = std::stoi(payload) & 0xFF;
        log_i("MQTT msg received: reset(0x%X)", flags);
        rainGauge.reset(flags);
        if (flags & 0x10)
        {
            lightning.reset();
        }
    }
    else if (topic == mqttTopics.subGetInc)
    {
        log_i("MQTT msg received: get_sensors_inc");
        cJSON *json = getSensorsIncJson(weatherSensor);
        publish(client, (Hostname + "/" + mqttTopics.pubInc).c_str(), json , 1, 0, "get_sensors_inc");
        cJSON_Delete(json);
    }
    else if (topic == mqttTopics.subGetExc)
    {
        log_i("MQTT msg received: get_sensors_exc");
        cJSON *json = getSensorsExcJson(weatherSensor);
        publish(client, (Hostname + "/" + mqttTopics.pubExc).c_str(), json, 1, 0, "get_sensors_exc");
        cJSON_Delete(json);
    }
    else if (topic == mqttTopics.subSetInc)
    {
        log_i("MQTT msg received: set_sensors_inc");
        setSensorsIncJson(payload);
    }
    else if (topic == mqttTopics.subSetExc)
    {
        log_i("MQTT msg received: set_sensors_exc");
        setSensorsExcJson(payload);
    }
    else
    {
        log_w("MQTT unknown msg received: %s", topic.c_str());
    }
}

// Publish weather data as MQTT message
void MQTTComm::publishWeatherdata(bool complete, bool retain)
{
    cJSON * jsonCombined = cJSON_CreateObject();
    cJSON * combinedStatus = cJSON_AddObjectToObject(jsonCombined, "status"); // FIXME CHECK
    cJSON * jsonSensor = cJSON_CreateObject();
    cJSON * jsonExtra = cJSON_CreateObject();
#define ROUND_1(x) (((int)((x)*10))/10.0)

    for (size_t i = 0; i < weatherSensor.sensor.size(); i++)
    {
        if (!weatherSensor.sensor[i].valid)
            continue;

        if (weatherSensor.sensor[i].w.rain_ok)
        {
            struct tm timeinfo;
            time_t now = time(nullptr);
            localtime_r(&now, &timeinfo);
            if (timeinfo.tm_year > 120) {
                rainGauge.update(now, weatherSensor.sensor[i].w.rain_mm, weatherSensor.sensor[i].startup);
            } else {
                // TODO check if running SNTP
                log_w("We are in year %d", timeinfo.tm_year + 1900);
            }
        }

        // Example:
        // {"ch":0,"battery_ok":1,"humidity":44,"wind_gust":1.2,"wind_avg":1.2,"wind_dir":150,"rain":146}
        JSON_ADD_NUMBER(jsonSensor, "id", weatherSensor.sensor[i].sensor_id);
        JSON_ADD_NUMBER(jsonSensor, "ch", weatherSensor.sensor[i].chan);
        JSON_ADD_NUMBER(jsonSensor, "battery_ok", weatherSensor.sensor[i].battery_ok);

#if defined(DATA_TIMESTAMP)
        {
            // Generate timestamp in ISO 8601 format
            time_t now = time(nullptr);
            struct tm timeinfo;
            gmtime_r(&now, &timeinfo); // Convert to UTC time
            char tbuf[25];
            strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo); // Format as ISO 8601
            JSON_ADD_STRING(jsonSensor, "timestamp", tbuf);
        }
#endif // DATA_TIMESTAMP

        if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_SOIL)
        {
            JSON_ADD_NUMBER(jsonSensor, "temp_c", weatherSensor.sensor[i].soil.temp_c);
            JSON_ADD_NUMBER(jsonSensor, "moisture", weatherSensor.sensor[i].soil.moisture);

            JSON_ADD_NUMBER(jsonCombined, "soil1_temp_c", weatherSensor.sensor[i].soil.temp_c);
            JSON_ADD_NUMBER(jsonCombined, "soil1_moisture", weatherSensor.sensor[i].soil.moisture);
            JSON_ADD_NUMBER(combinedStatus, "soil1_batt_ok", weatherSensor.sensor[i].battery_ok ? 1 : 0);
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_LIGHTNING)
        {
            JSON_ADD_NUMBER(jsonSensor, "lightning_count", weatherSensor.sensor[i].lgt.strike_count);
            JSON_ADD_NUMBER(jsonSensor, "lightning_distance_km", weatherSensor.sensor[i].lgt.distance_km);
            char lgtUnknown1[12], lgtUnknown2[12];
            snprintf(lgtUnknown1, sizeof(lgtUnknown1), "0x%x", weatherSensor.sensor[i].lgt.unknown1);
            snprintf(lgtUnknown2, sizeof(lgtUnknown2), "0x%x", weatherSensor.sensor[i].lgt.unknown2);
            JSON_ADD_STRING(jsonSensor, "lightning_unknown1", lgtUnknown1);
            JSON_ADD_STRING(jsonSensor, "lightning_unknown2", lgtUnknown2);

            struct tm timeinfo;
            time_t now = time(nullptr);
            localtime_r(&now, &timeinfo);
            lightning.update(
                now,
                weatherSensor.sensor[i].lgt.strike_count,
                weatherSensor.sensor[i].lgt.distance_km,
                weatherSensor.sensor[i].startup);
            JSON_ADD_NUMBER(jsonSensor, "lightning_hr", lightning.pastHour());
            int events;
            time_t timestamp;
            uint8_t distance;
            if (lightning.lastEvent(timestamp, events, distance))
            {
                char tbuf[25];
                struct tm timeinfo;
                gmtime_r(&timestamp, &timeinfo);
                strftime(tbuf, 25, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
                JSON_ADD_STRING(jsonSensor, "lightning_event_time", tbuf);
                JSON_ADD_NUMBER(jsonSensor, "lightning_event_count", events);
                JSON_ADD_NUMBER(jsonSensor, "lightning_event_distance_km", distance);

                JSON_ADD_NUMBER(jsonCombined, "lgt_ev_time", timestamp);
                JSON_ADD_NUMBER(jsonCombined, "lgt_ev_events", events);
                JSON_ADD_NUMBER(jsonCombined, "lgt_ev_dist_km", distance);
                JSON_ADD_NUMBER(combinedStatus, "ls_batt_ok", weatherSensor.sensor[i].battery_ok ? 1 : 0);
            }
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_LEAKAGE)
        {
            // Water Leakage Sensor
            JSON_ADD_NUMBER(jsonSensor, "leakage", weatherSensor.sensor[i].leak.alarm ? 1 : 0);
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_AIR_PM)
        {
            // Air Quality (Particular Matter) Sensor
            if (!weatherSensor.sensor[i].pm.pm_1_0_init)
            {
                JSON_ADD_NUMBER(jsonSensor, "pm1_0_ug_m3", weatherSensor.sensor[i].pm.pm_1_0);
            }
            if (!weatherSensor.sensor[i].pm.pm_2_5_init)
            {
                JSON_ADD_NUMBER(jsonSensor, "pm2_5_ug_m3", weatherSensor.sensor[i].pm.pm_2_5);
            }
            if (!weatherSensor.sensor[i].pm.pm_10_init)
            {
                JSON_ADD_NUMBER(jsonSensor, "pm10_ug_m3", weatherSensor.sensor[i].pm.pm_10);
            }
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_CO2)
        {
            // CO2 Sensor
            if (!weatherSensor.sensor[i].co2.co2_init)
            {
                JSON_ADD_NUMBER(jsonSensor, "co2_ppm", weatherSensor.sensor[i].co2.co2_ppm);
            }
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_HCHO_VOC)
        {
            // HCHO / VOC Sensor
            if (!weatherSensor.sensor[i].voc.hcho_init)
            {
                JSON_ADD_NUMBER(jsonSensor, "hcho_ppb", weatherSensor.sensor[i].voc.hcho_ppb);
            }
            if (!weatherSensor.sensor[i].voc.voc_init)
            {
                JSON_ADD_NUMBER(jsonSensor, "voc", weatherSensor.sensor[i].voc.voc_level);
            }
        }
        else if ((weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER0) ||
                 (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER1) ||
                 (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER3) ||
                 (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER8) ||
                 (weatherSensor.sensor[i].s_type == SENSOR_TYPE_THERMO_HYGRO) ||
                 (weatherSensor.sensor[i].s_type == SENSOR_TYPE_POOL_THERMO))
        {
            if ((weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER0) ||
                (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER1) ||
                (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER3) ||
                (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER8))
            {
                JSON_ADD_NUMBER(combinedStatus, "ws_batt_ok", weatherSensor.sensor[i].battery_ok ? 1 : 0);
            }
            if (weatherSensor.sensor[i].w.temp_ok || complete)
            {
                JSON_ADD_NUMBER(jsonSensor, "temp_c", ROUND_1(weatherSensor.sensor[i].w.temp_c));
                JSON_ADD_NUMBER(jsonCombined, "ws_temp_c", ROUND_1(weatherSensor.sensor[i].w.temp_c));
            }
            if (weatherSensor.sensor[i].w.humidity_ok || complete)
            {
                JSON_ADD_NUMBER(jsonSensor, "humidity", weatherSensor.sensor[i].w.humidity);
                JSON_ADD_NUMBER(jsonCombined, "ws_humidity", weatherSensor.sensor[i].w.humidity);
            }
            if (weatherSensor.sensor[i].w.wind_ok || complete)
            {
                JSON_ADD_NUMBER(jsonSensor, "wind_gust", weatherSensor.sensor[i].w.wind_gust_meter_sec);
                JSON_ADD_NUMBER(jsonSensor, "wind_avg", weatherSensor.sensor[i].w.wind_avg_meter_sec);
                JSON_ADD_NUMBER(jsonSensor, "wind_dir", weatherSensor.sensor[i].w.wind_direction_deg);
                JSON_ADD_NUMBER(jsonCombined, "ws_wind_gust_ms", weatherSensor.sensor[i].w.wind_gust_meter_sec);
                JSON_ADD_NUMBER(jsonCombined, "ws_wind_avg_ms", weatherSensor.sensor[i].w.wind_avg_meter_sec);
                JSON_ADD_NUMBER(jsonCombined, "ws_wind_dir_deg", weatherSensor.sensor[i].w.wind_direction_deg);
            }
            if (weatherSensor.sensor[i].w.wind_ok)
            {
                char buf[4];
                JSON_ADD_STRING(jsonExtra, "wind_dir_txt", winddir_flt_to_str(weatherSensor.sensor[i].w.wind_direction_deg, buf, sizeof(buf)));
                JSON_ADD_NUMBER(jsonExtra, "wind_gust_bft", windspeed_ms_to_bft(weatherSensor.sensor[i].w.wind_gust_meter_sec));
                JSON_ADD_NUMBER(jsonExtra, "wind_avg_bft", windspeed_ms_to_bft(weatherSensor.sensor[i].w.wind_avg_meter_sec));
            }
            if ((weatherSensor.sensor[i].w.temp_ok) && (weatherSensor.sensor[i].w.humidity_ok))
            {
                JSON_ADD_NUMBER(jsonExtra, "dewpoint_c", calcdewpoint(weatherSensor.sensor[i].w.temp_c, weatherSensor.sensor[i].w.humidity));

                if (weatherSensor.sensor[i].w.wind_ok)
                {
                    JSON_ADD_NUMBER(jsonExtra, "perceived_temp_c", ROUND_1(perceived_temperature(weatherSensor.sensor[i].w.temp_c, weatherSensor.sensor[i].w.wind_avg_meter_sec, weatherSensor.sensor[i].w.humidity)));
                }
                if (weatherSensor.sensor[i].w.tglobe_ok)
                {
                    float t_wet = calcnaturalwetbulb(weatherSensor.sensor[i].w.temp_c, weatherSensor.sensor[i].w.humidity);
                    JSON_ADD_NUMBER(jsonExtra, "wgbt", calcwbgt(t_wet, weatherSensor.sensor[i].w.tglobe_c, weatherSensor.sensor[i].w.temp_c));

                }
            }
            if (weatherSensor.sensor[i].w.uv_ok || complete)
            {
                JSON_ADD_NUMBER(jsonSensor, "uv", ROUND_1(weatherSensor.sensor[i].w.uv));
                JSON_ADD_NUMBER(jsonCombined, "ws_uv", ROUND_1(weatherSensor.sensor[i].w.uv));
            }
            if (weatherSensor.sensor[i].w.light_ok || complete)
            {
                JSON_ADD_NUMBER(jsonSensor, "light_lx", weatherSensor.sensor[i].w.light_lux);
                JSON_ADD_NUMBER(jsonCombined, "ws_light_lx", weatherSensor.sensor[i].w.light_lux);
            }
            if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER8)
            {
                if (weatherSensor.sensor[i].w.tglobe_ok || complete)
                {
                    JSON_ADD_NUMBER(jsonSensor, "t_globe_c", weatherSensor.sensor[i].w.tglobe_c);
                    JSON_ADD_NUMBER(jsonCombined, "ws_t_globe_c", weatherSensor.sensor[i].w.tglobe_c);
                }
            }
            if (weatherSensor.sensor[i].w.rain_ok || complete)
            {
                JSON_ADD_NUMBER(jsonSensor, "rain", ROUND_1(weatherSensor.sensor[i].w.rain_mm));
                JSON_ADD_NUMBER(jsonSensor, "rain_h", rainGauge.pastHour());
                JSON_ADD_NUMBER(jsonSensor, "rain_d24h", rainGauge.past24Hours());
                JSON_ADD_NUMBER(jsonSensor, "rain_d", rainGauge.currentDay());
                JSON_ADD_NUMBER(jsonSensor, "rain_w", rainGauge.currentWeek());
                JSON_ADD_NUMBER(jsonSensor, "rain_m", rainGauge.currentMonth());
                JSON_ADD_NUMBER(jsonCombined, "ws_rain_mm", ROUND_1(weatherSensor.sensor[i].w.rain_mm));
                JSON_ADD_NUMBER(jsonCombined, "ws_rain_hourly_mm", rainGauge.pastHour());
                JSON_ADD_NUMBER(jsonCombined, "ws_rain_24h_mm", rainGauge.past24Hours());
                JSON_ADD_NUMBER(jsonCombined, "ws_rain_daily_mm", rainGauge.currentDay());
                JSON_ADD_NUMBER(jsonCombined, "ws_rain_weekly_mm", rainGauge.currentWeek());
                JSON_ADD_NUMBER(jsonCombined, "ws_rain_monthly_mm", rainGauge.currentMonth());
            }
        }

        // Try to map sensor ID to name to make MQTT topic explanatory
        char sensor_str[32];
        sensorName(weatherSensor.sensor[i].sensor_id, sensor_str, sizeof(sensor_str));
        std::string sensorName = this->sensorName(weatherSensor.sensor[i].sensor_id);
        std::string topicPrefix = Hostname + "/" + sensorName + "/";

        // sensor data
        publish(client, topicPrefix + mqttTopics.pubData, jsonSensor, 0, retain, "jsonSensor");

        // sensor specific RSSI
        char rssiStr[12];
        snprintf(rssiStr, sizeof(rssiStr), "%.1f", weatherSensor.sensor[i].rssi);
        publish(client, (topicPrefix + mqttTopics.pubRssi).c_str(), rssiStr, 0, 0, false);

        // extra data
        if (jsonExtra != 0) // FIXME check for empty
        {
            // extra data
            JSON_ADD_NUMBER(jsonExtra, "id", weatherSensor.sensor[i].sensor_id);
            JSON_ADD_NUMBER(jsonExtra, "ch", weatherSensor.sensor[i].chan);
            publish(client, Hostname + "/" + mqttTopics.pubExtra, jsonExtra, 0, retain, "jsonExtra");
        }
    } // for (int i=0; i<weatherSensor.sensor.size(); i++)

    publish(client, Hostname + "/" + mqttTopics.pubCombined, jsonCombined, 0, retain, "jsonCombined");

    cJSON_Delete(jsonSensor);
    cJSON_Delete(jsonExtra);
    cJSON_Delete(jsonCombined);
    return;
}

// Publish radio receiver info as JSON string via MQTT
// - RSSI: Received Signal Strength Indication
void MQTTComm::publishRadio(void)
{
    cJSON * payload = cJSON_CreateObject();

    JSON_ADD_NUMBER(payload, "rssi", weatherSensor.rssi);
    publish(client, Hostname + "/" + mqttTopics.pubRadio, payload, 0, false, "radio");

    cJSON_Delete(payload);
    return;
}

// Home Assistant Auto-Discovery
void MQTTComm::haAutoDiscovery(void)
{
    for (size_t i = 0; i < weatherSensor.sensor.size(); i++)
    {
        uint32_t sensor_id = weatherSensor.sensor[i].sensor_id;
        if (!weatherSensor.sensor[i].valid)
            continue;

        char sensor_str[32];
        bool named = sensorName(weatherSensor.sensor[i].sensor_id, sensor_str, sizeof(sensor_str));
        // Stack-allocated topic buffers avoid heap fragmentation from String concatenation.
        char topicData[128], topicRssi[128], topicExtra[128];
        snprintf(topicData,  sizeof(topicData),  "%s/%s/data",  Hostname.c_str(), sensor_str);
        snprintf(topicRssi,  sizeof(topicRssi),  "%s/%s/rssi",  Hostname.c_str(), sensor_str);
        snprintf(topicExtra, sizeof(topicExtra), "%s/extra",    Hostname.c_str());
        if ((weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER0) ||
            (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER1) ||
            (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER3) ||
            (weatherSensor.sensor[i].s_type == SENSOR_TYPE_WEATHER8))
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "weather_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Weather Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "Outside Temperature", sensor_id, "temperature", "°C", topicData, "temp_c");
            publishAutoDiscovery(info, "Outside Humidity", sensor_id, "humidity", "%", topicData, "humidity");
            if (weatherSensor.sensor[i].w.tglobe_ok)
            {
                publishAutoDiscovery(info, "Globe Temperature", sensor_id, "temperature", "°C", topicData, "tglobe_c");
            }
            if (weatherSensor.sensor[i].w.uv_ok)
            {
                publishAutoDiscovery(info, "UV Index", sensor_id, NULL, "UV", topicData, "uv");
            }
            if (weatherSensor.sensor[i].w.light_ok)
            {
                publishAutoDiscovery(info, "Light Lux", sensor_id, "illuminance", "lx", topicData, "light_lx");
            }
            if (weatherSensor.sensor[i].w.rain_ok)
            {
                publishAutoDiscovery(info, "Rainfall", sensor_id, "precipitation", "mm", topicData, "rain");
                publishAutoDiscovery(info, "Rainfall Hourly", sensor_id, "precipitation", "mm", topicData, "rain_h");
                publishAutoDiscovery(info, "Rainfall 24h", sensor_id, "precipitation", "mm", topicData, "rain_d24h");
                publishAutoDiscovery(info, "Rainfall Daily", sensor_id, "precipitation", "mm", topicData, "rain_d");
                publishAutoDiscovery(info, "Rainfall Weekly", sensor_id, "precipitation", "mm", topicData, "rain_w");
                publishAutoDiscovery(info, "Rainfall Monthly", sensor_id, "precipitation", "mm", topicData, "rain_m");
            }
            if (weatherSensor.sensor[i].w.wind_ok)
            {
                publishAutoDiscovery(info, "Wind Direction", sensor_id, "wind_direction", "°", topicData, "wind_dir");
                publishAutoDiscovery(info, "Wind Gust Speed", sensor_id, "wind_speed", "m/s", topicData, "wind_gust");
                publishAutoDiscovery(info, "Wind Average Speed", sensor_id, "wind_speed", "m/s", topicData, "wind_avg");
                publishAutoDiscovery(info, "Wind Gust Speed (Beaufort)", sensor_id, NULL, "Beaufort", topicExtra, "wind_gust_bft");
                publishAutoDiscovery(info, "Wind Average Speed (Beaufort)", sensor_id, NULL, "Beaufort", topicExtra, "wind_avg_bft");
                publishAutoDiscovery(info, "Wind Direction (Cardinal)", sensor_id, NULL, NULL, topicExtra, "wind_dir_txt");
            }
            if (weatherSensor.sensor[i].w.wind_ok &&
                weatherSensor.sensor[i].w.temp_ok &&
                weatherSensor.sensor[i].w.humidity_ok)
            {
                publishAutoDiscovery(info, "Dewpoint", sensor_id, "temperature", "°C", topicExtra, "dewpoint_c");
                publishAutoDiscovery(info, "Perceived Temperature", sensor_id, "temperature", "°C", topicExtra, "perceived_temp_c");
                if (weatherSensor.sensor[i].w.tglobe_ok)
                {
                    publishAutoDiscovery(info, "WGBT", sensor_id, "temperature", "°C", topicExtra, "wgbt");
                }
            }
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_SOIL)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "soil_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Soil Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "Soil Temperature", sensor_id, "temperature", "°C", topicData, "temp_c");
            publishAutoDiscovery(info, "Soil Moisture", sensor_id, "moisture", "%", topicData, "moisture");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_THERMO_HYGRO)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "thermo_hygro_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Thermo-Hygrometer Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "Temperature", sensor_id, "temperature", "°C", topicData, "temp_c");
            publishAutoDiscovery(info, "Humidity", sensor_id, "humidity", "%", topicData, "humidity");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_POOL_THERMO)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "pool_thermo_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Pool Thermometer",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "Pool Temperature", sensor_id, "temperature", "°C", topicData, "temp_c");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_AIR_PM)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "air_pm_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Air Quality (PM) Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "PM1.0", sensor_id, "pm1", "µg/m³", topicData, "pm1_0_ug_m3");
            publishAutoDiscovery(info, "PM2.5", sensor_id, "pm25", "µg/m³", topicData, "pm2_5_ug_m3");
            publishAutoDiscovery(info, "PM10", sensor_id, "pm10", "µg/m³", topicData, "pm10_ug_m3");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_LIGHTNING)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "lightning_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Lightning Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "Lightning Count", sensor_id, NULL, "", topicData, "lightning_count");
            publishAutoDiscovery(info, "Lightning Distance", sensor_id, "distance", "km", topicData, "lightning_distance_km");
            publishAutoDiscovery(info, "Lightning Hour", sensor_id, NULL, "", topicData, "lightning_hr");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_LEAKAGE)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "leakage_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Leakage Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "Leakage Alarm", sensor_id, "enum", "", topicData, "leakage");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_CO2)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "co2_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "CO2 Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "CO2", sensor_id, "co2", "ppm", topicData, "co2_ppm");
        }
        else if (weatherSensor.sensor[i].s_type == SENSOR_TYPE_HCHO_VOC)
        {
            char identifier[32];
            snprintf(identifier, sizeof(identifier), "hcho_voc_%08x", (unsigned)sensor_id);
            struct sensor_info info = {
                .manufacturer = "Bresser",
                .model = "Air Quality (HCHO/VOC) Sensor",
                .identifier = identifier,
                .display_name = named ? sensor_str : nullptr};

            publishAutoDiscovery(info, "Battery", sensor_id, "battery", "%", topicData, "battery_ok");
            publishAutoDiscovery(info, "RSSI", sensor_id, "signal_strength", "dBm", topicRssi, "rssi");
            publishAutoDiscovery(info, "HCHO", sensor_id, "hcho", "ppb", topicData, "hcho_ppb");
            publishAutoDiscovery(info, "VOC", sensor_id, "voc", "", topicData, "voc");
        }
    } // for (int i=0; i<weatherSensor.sensor.size(); i++)

    publishControlDiscovery("Sensor Exclude List", "sensors_exc");
    publishControlDiscovery("Sensor Include List", "sensors_inc");
    publishStatusDiscovery("Receiver Status", "status");
}

// Publish discovery message for MQTT node status
void MQTTComm::publishStatusDiscovery(const char* name, const char* topic)
{
    char discoveryTopic[256];

    cJSON * doc = cJSON_CreateObject();
    JSON_ADD_STRING(doc, "name", name);
    char uniqueId[80];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", Hostname.c_str(), topic);
    JSON_ADD_STRING(doc, "unique_id", uniqueId);
    char stateTopic[80];
    snprintf(stateTopic, sizeof(stateTopic), "%s/%s", Hostname.c_str(), topic);
    JSON_ADD_STRING(doc, "state_topic", stateTopic);
    JSON_ADD_STRING(doc, "value_template", "{{ value }}");
    JSON_ADD_STRING(doc, "icon", "mdi:wifi");
    cJSON * device = cJSON_AddObjectToObject(doc, "device"); // FIXME CHECK
    char identifiers[48];
    snprintf(identifiers, sizeof(identifiers), "%s_1", Hostname.c_str());
    JSON_ADD_STRING(device, "identifiers", identifiers);
    JSON_ADD_STRING(device, "name", "Weather Sensor Receiver");

    snprintf(discoveryTopic, sizeof(discoveryTopic), "homeassistant/sensor/%s/%s/config",
             Hostname.c_str(), topic);
    publish(client, discoveryTopic, doc, 0, false, "discovery");

    cJSON_Delete(doc);
    return;
}

// Publish discovery messages for receiver control
void MQTTComm::publishControlDiscovery(const char* name, const char* topic)
{
    char discoveryTopic[256];

    // Sensor discovery
    cJSON * doc = cJSON_CreateObject();
    JSON_ADD_STRING(doc, "name", name);
    char uniqueId[80];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", Hostname.c_str(), topic);
    JSON_ADD_STRING(doc, "unique_id", uniqueId);
    char stateTopic[80];
    snprintf(stateTopic, sizeof(stateTopic), "%s/%s", Hostname.c_str(), topic);
    JSON_ADD_STRING(doc, "state_topic", stateTopic);
    JSON_ADD_STRING(doc, "value_template", "{{ value_json.ids }}");
    JSON_ADD_STRING(doc, "icon", "mdi:code-array");

    cJSON * device = cJSON_AddObjectToObject(doc, "device"); // FIXME CHECK
    char identifiers[48];
    snprintf(identifiers, sizeof(identifiers), "%s_1", Hostname.c_str());
    JSON_ADD_STRING(device, "identifiers", identifiers);
    JSON_ADD_STRING(device, "name", "Weather Sensor Receiver");

    snprintf(discoveryTopic, sizeof(discoveryTopic), "homeassistant/sensor/%s/%s/config",
             Hostname.c_str(), topic);
    publish(client, discoveryTopic, doc, 0, true, "control_discovery");

    // Button discovery
    cJSON * docButton = cJSON_CreateObject();
    char buttonName[80];
    snprintf(buttonName, sizeof(buttonName), "Get %s", name);
    JSON_ADD_STRING(docButton, "name", buttonName);
    JSON_ADD_STRING(docButton, "platform", "button");
    char buttonUniqueId[80];
    snprintf(buttonUniqueId, sizeof(buttonUniqueId), "%s_get_%s", Hostname.c_str(), topic);
    JSON_ADD_STRING(docButton, "unique_id", buttonUniqueId);
    char buttonCmdTopic[80];
    snprintf(buttonCmdTopic, sizeof(buttonCmdTopic), "%s/get_%s", Hostname.c_str(), topic);
    JSON_ADD_STRING(docButton, "command_topic", buttonCmdTopic);
    JSON_ADD_STRING(docButton, "icon", "mdi:information");
    JSON_ADD_NUMBER(docButton, "retain", true);
    JSON_ADD_NUMBER(docButton, "qos", 1);

    cJSON * deviceBtn = cJSON_AddObjectToObject(docButton, "device"); // FIXME CHECK
    JSON_ADD_STRING(deviceBtn, "identifiers", identifiers);
    JSON_ADD_STRING(deviceBtn, "name", "Weather Sensor Receiver");

    snprintf(discoveryTopic, sizeof(discoveryTopic), "homeassistant/button/%s/get_%s/config",
             Hostname.c_str(), topic);
    publish(client, discoveryTopic, docButton, 0, false, "button");

    cJSON_Delete(doc);
    cJSON_Delete(docButton);
    return;
}

// Publish auto-discovery configuration for Home Assistant
void MQTTComm::publishAutoDiscovery(const struct sensor_info info, const char *sensor_name, const uint32_t sensor_id, const char *device_class, const char *unit, const char *state_topic, const char *value_json)
{
    cJSON * doc = cJSON_CreateObject();

    JSON_ADD_STRING(doc, "name", sensor_name);
    if (device_class != NULL)
        JSON_ADD_STRING(doc, "device_class", device_class);
    char uniqueId[64];
    snprintf(uniqueId, sizeof(uniqueId), "%08x_%s", (unsigned)sensor_id, value_json);
    JSON_ADD_STRING(doc, "unique_id", uniqueId);
    JSON_ADD_STRING(doc, "state_topic", state_topic);
    char availTopic[64];
    snprintf(availTopic, sizeof(availTopic), "%s/status", Hostname.c_str());
    JSON_ADD_STRING(doc, "availability_topic", availTopic);
    JSON_ADD_STRING(doc, "payload_not_available", "dead"); // default: "offline"
    if (unit != NULL)
        JSON_ADD_STRING(doc, "unit_of_measurement", unit);
    char valTmpl[128];
    if (device_class != NULL)
    {
        if (strcmp(device_class, "battery") == 0)
        {
            snprintf(valTmpl, sizeof(valTmpl), "{{ (value_json.%s | float) * 100.0 }}", value_json);
            JSON_ADD_STRING(doc, "value_template", valTmpl);
        }
        else if (strcmp(device_class, "signal_strength") == 0)
        {
            JSON_ADD_STRING(doc, "value_template", "{{ value }}");
        }
        else
        {
            snprintf(valTmpl, sizeof(valTmpl), "{{ value_json.%s }}", value_json);
            JSON_ADD_STRING(doc, "value_template", valTmpl);
        }
    } else {
        snprintf(valTmpl, sizeof(valTmpl), "{{ value_json.%s }}", value_json);
        JSON_ADD_STRING(doc, "value_template", valTmpl);
    }

    cJSON * device = cJSON_AddObjectToObject(doc, "device"); // FIXME CHECK
    JSON_ADD_STRING(device, "identifiers", info.identifier);
    char deviceName[80];
    if (info.display_name && info.display_name[0] != '\0')
        snprintf(deviceName, sizeof(deviceName), "%s", info.display_name);
    else
        snprintf(deviceName, sizeof(deviceName), "%s %s", info.manufacturer, info.model);
    JSON_ADD_STRING(device, "name", deviceName);
    if (info.model[0] != '\0')
        JSON_ADD_STRING(device, "model", info.model);
    if (info.manufacturer[0] != '\0')
        JSON_ADD_STRING(device, "manufacturer", info.manufacturer);

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "homeassistant/sensor/%08x_%s/config", (unsigned)sensor_id, value_json);
    publish(client, discTopic, doc, 0, true, "auto_discovery");
    log_d("Published auto-discovery configuration for %s", sensor_name);

    cJSON_Delete(doc);
    return;
}
