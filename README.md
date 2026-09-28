# BresserWisser
Bresser weather station receiver with RFM69 receiver.
Posts weather data to MQTT topic.

Run the usual commands
```
idf.py update-dependencies
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor -p /dev/ttyACM0 
```
