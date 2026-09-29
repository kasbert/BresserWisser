# BresserWisser
Bresser weather station receiver with SX1262 or RFM69 module.
Posts weather data to MQTT topic.
Use esp-idf instead of Arduino.

Run the usual commands
```
idf.py update-dependencies
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor -p /dev/ttyACM0 
```

Code largely copied from https://github.com/matthias-bs/BresserWeatherSensorReceiver