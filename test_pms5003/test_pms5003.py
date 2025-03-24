import serial
import struct
import time
import sys
from datetime import datetime


PMS5003_MSGLEN = 32
PMS5003_BAUDRATE = 9600
PMS5003_HEAD1 = 0x42
PMS5003_HEAD2 = 0x4D
SENSOR_PM_UPDATE_INTERVAL = 60

def read_pms5003(port):
    """
    Reads data from the PMS5003 air quality sensor via a USB serial connection.
    
    :param port: Serial port where the sensor is connected
    """
    try:
        ser = serial.Serial(port, PMS5003_BAUDRATE, timeout=2)
        data = ser.read(PMS5003_MSGLEN)  # Read 32 bytes (full message length for PMS5003)
            
        if len(data) >= PMS5003_MSGLEN and data[0] == PMS5003_HEAD1 and data[1] == PMS5003_HEAD2:
#            frame = struct.unpack('!2B13H2B', data)  # Unpack the received data
            pm1_0, pm2_5, pm10 = data[3], data[4], data[5]
        else:
            pm1_0 = pm2_5 = pm10 = -1
    except serial.SerialException as e:
        pm1_0 = pm2_5 = pm10 = -100
        print(f"Error: {e}")
    else:
        ser.close()
    return pm1_0, pm2_5, pm10

# Example usage
sport = sys.argv[1]
while(True):
    current_time = datetime.now().strftime("%H:%M:%S")
    pm1_0, pm2_5, pm10 = read_pms5003(sport)
    if not (pm1_0 == pm2_5 == pm10):
        print(f"{current_time}\tPM1.0: {pm1_0} µg/m³, PM2.5: {pm2_5} µg/m³, PM10: {pm10} µg/m³")
    else:
        print("{current_time}\tPM1.0: X µg/m³, PM2.5: X µg/m³, PM10: X µg/m³")
    time.sleep(SENSOR_PM_UPDATE_INTERVAL)