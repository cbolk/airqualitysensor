# PMS_5003
Sensor for PM1.0, PM2.5, PM10 concentration

Sensor library (''pms5003.c'' and ''pms5003.h'') taken from [https://github.com/ttrinh3/bme680-pms/tree/lora-680-280-same-time][https://github.com/ttrinh3/bme680-pms/tree/lora-680-280-same-time], but fixed a couple of points because it did not read the selected uart (it was always looking at UART1)

Modified also the measurement to actually exploit the possibility to make an average of the last ''num'' readings.
