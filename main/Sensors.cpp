///////////////////////////////////////////////////////////////////////////////////////////////////
// C++ <-> C stub
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "Sensors.h"
#include "MQTTComm.h"

//static const char *TAG = "SENSORS";

struct Sensors {
    WeatherSensor weatherSensor;
    MQTTComm comm;

    Sensors(): comm(weatherSensor) {};
};

extern "C"
void * Sensors_new() {
    Sensors *s = new Sensors();
    s->weatherSensor.begin(10, true);
    return (void*)s; 
}

extern "C"
bool Sensors_decodeMessage(void *s_, const uint8_t *msg, uint8_t msgSize, int16_t rssi) {
    auto s = reinterpret_cast<Sensors*>(s_);
    if (s->weatherSensor.decodeMessage(msg, msgSize, rssi) == DECODE_OK) {
        static int count = 0;
        // FIXME kludge. use time
        if (++count == 3) {
            s->comm.haAutoDiscovery();
        }
        if (count > 100) {
            count = 0;
        }
        return true;
    }
    return false;
}

extern "C"
void Sensors_mqttConnected(void *s_, esp_mqtt_client_handle_t handle) {
    auto s = reinterpret_cast<Sensors*>(s_);
    s->comm.setClient(handle);
}

extern "C"
void Sensors_mqttData(void *s_, const char *topic, int topic_len, const char *data, int data_len) {
    auto s = reinterpret_cast<Sensors*>(s_);
    s->comm.messageReceived(std::string(topic, topic_len), std::string(data, data_len));
}

extern "C"
bool Sensors_publishWeatherdata(void *s_) {
    auto s = reinterpret_cast<Sensors*>(s_);
    s->comm.publishWeatherdata(false, false);
    return true;
}
