set(srcs "main.c" "radio.c" "net.c" "sntp.c" "Sensors.cpp" "MQTTComm.cpp")

idf_component_register(SRCS "${srcs}"
	PRIV_REQUIRES rf69 esp_driver_gpio cjson nvs_flash mqtt BresserWeatherSensorReceiver wpa_supplicant lwip esp_wifi mdns
	INCLUDE_DIRS ".")
