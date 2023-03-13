# Flutuador-V0
```
Diretivas de compilação:
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
lib_deps = 
	adafruit/Adafruit ADS1X15@^2.4.0
	adafruit/Adafruit BME280 Library@^2.2.2
	adafruit/Adafruit BMP280 Library@^2.6.2
	adafruit/RTClib@^2.0.2
	greenponik/DFRobot_ESP_PH_WITH_ADC_BY_GREENPONIK@^1.2.3
	milesburton/DallasTemperature@^3.9.1
	sandeepmistry/LoRa@^0.8.0
	mcci-catena/MCCI LoRaWAN LMIC library@^4.1.1
build_flags =
    -D ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS
    -D CFG_au915=1
    -D CFG_sx1276_radio=1
    -D LMIC_LORAWAN_SPEC_VERSION=LMIC_LORAWAN_SPEC_VERSION_1_0_3
    -D LMIC_PRINTF_TO=Serial    
    -D LMIC_ENABLE_arbitrary_clock_error=1
```
