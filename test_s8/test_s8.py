import serial
import time
import sys
from datetime import datetime

'''
Daily AQI Color | Levels of Concern              | Values of Index     | Description of Air Quality
---------------------------------------------------------------------------------------------------------------------------------------------------
Green           | Good                           | 0 to 50             | Air quality is satisfactory, and air pollution poses little or no risk.
Yellow          | Moderate                       | 51 to 100           | Air quality is acceptable. However, there may be a risk for some people, particularly those who are unusually sensitive to air pollution.
Orange          | Unhealthy for Sensitive Groups | 101 to 150          | Members of sensitive groups may experience health effects. The general public is less likely to be affected.
Red             | Unhealthy                      | 151 to 200          | Some members of the general public may experience health effects; members of sensitive groups may experience more serious health effects.
Purple          | Very Unhealthy                 | 201 to 300          | Health alert: The risk of health effects is increased for everyone.
Maroon          | Hazardous                      | 301 and higher      | Health warning of emergency conditions: everyone is more likely to be affected.
'''

SENSOR_TVOC_UPDATE_INTERVAL = 1     # s */
SENSOR_CO2_UPDATE_INTERVAL = 4      # s */
SENSOR_PM_UPDATE_INTERVAL = 2       # s */
SENSOR_TEMP_HUM_UPDATE_INTERVAL = 6 # s */
DISPLAY_DELAY_SHOW_CONTENT_MS = 2   # s */


#Max length of buffer for communication with the sensor
S8_LEN_BUF_MSG = 20
S8_BAUDRATE = 9600
S8_TIMEOUT = 0.1 #5000 # Timeout for communication in milliseconds
S8_LEN_FIRMVER = 10 # Length of software version

S8_MSG_RESPONSE_LEN = 7
S8_MSG_RESPONSE_LEN_C02 = 7
S8_MSG_RESPONSE_LEN_CAL = 8

# ModbusAddr = {
#     MODBUS_ANY_ADDRESS: 0XFE,                 # S8 uses any address
#     MODBUS_FUNC_READ_HOLDING_REGISTERS: 0X03, # Read holding registers (HR)
#     MODBUS_FUNC_READ_INPUT_REGISTERS: 0x04,   # Read input registers (IR)
#     MODBUS_FUNC_WRITE_SINGLE_REGISTER: 0x06
# }
# ModbusRegInput = {
#     MODBUS_IR1: 0x0000,  # MeterStatus
#     MODBUS_IR2: 0x0001,  # AlarmStatus
#     MODBUS_IR3: 0x0002,  # OutputStatus
#     MODBUS_IR4: 0x0003,  # Space CO2
#     MODBUS_IR22: 0x0015, # PWM Output
#     MODBUS_IR26: 0x0019, # Sensor Type ID High
#     MODBUS_IR27: 0x001A, # Sensor Type ID Low
#     MODBUS_IR28: 0x001B, # Memory Map version
#     MODBUS_IR29: 0x001C, # FW version Main.Sub
#     MODBUS_IR30: 0x001D, # Sensor ID High
#     MODBUS_IR31: 0x001E  # Sensor ID Low
# }
# ModbusRegHolding = {
#     MODBUS_HR1: 0x0000,  # Acknowledgement Register
#     MODBUS_HR2: 0x0001,  # Special Command Register
#     MODBUS_HR32: 0x001F # ABC Period
# }
# OutputStatus = {
#     S8_MASK_OUTPUT_ALARM: 0x0001, # Alarm output status (inverted due to Open Collector)
#     S8_MASK_OUTPUT_PWM: 0x0002 # PWM output status (=1 -> full output)
# }
# AcknowledgementFLags = {
#     S8_MASK_CO2_BACKGROUND_CALIBRATION: 0x0020, # CO2 Background calibration performed = 1
#     S8_MASK_CO2_NITROGEN_CALIBRATION: 0x0040 # CO2 Nitrogen calibration performed = 1
# }
# CalibrationSpecialCommand = {
#     S8_CO2_BACKGROUND_CALIBRATION: 0x7C06, # CO2 Background calibration
#     S8_CO2_ZERO_CALIBRATION: 0x7C07       # CO2 Zero calibration
# }

# Status = {
#     S8_MASK_METER_FATAL_ERROR: 0x0001,             # Fatal error
#     S8_MASK_METER_OFFSET_REGULATION_ERROR: 0x0002, # Offset regulation error
#     S8_MASK_METER_ALGORITHM_ERROR: 0x0004,         # Algorithm error
#     S8_MASK_METER_OUTPUT_ERROR: 0x0008,            # Output error
#     S8_MASK_METER_SELF_DIAG_ERROR: 0x0010,         # Self diagnostics error
#     S8_MASK_METER_OUT_OF_RANGE: 0x0020,            # Out of range
#     S8_MASK_METER_MEMORY_ERROR: 0x0040,            # Memory error
#     S8_MASK_METER_ANY_ERROR: 0x007F # Mask to detect the previous errors
#                                       # (fatal error ... memory error)
# }


MODBUS_ANY_ADDRESS = 0XFE                 # S8 uses any address
MODBUS_FUNC_READ_HOLDING_REGISTERS = 0X03 # Read holding registers (HR)
MODBUS_FUNC_READ_INPUT_REGISTERS = 0x04   # Read input registers (IR)
MODBUS_FUNC_WRITE_SINGLE_REGISTER = 0x06

MODBUS_IR1 = 0x0000  # MeterStatus
MODBUS_IR2 = 0x0001  # AlarmStatus
MODBUS_IR3 = 0x0002  # OutputStatus
MODBUS_IR4 = 0x0003  # Space CO2
MODBUS_IR22 = 0x0015 # PWM Output
MODBUS_IR26 = 0x0019 # Sensor Type ID High
MODBUS_IR27 = 0x001A # Sensor Type ID Low
MODBUS_IR28 = 0x001B # Memory Map version
MODBUS_IR29 = 0x001C # FW version Main.Sub
MODBUS_IR30 = 0x001D # Sensor ID High
MODBUS_IR31 = 0x001E  # Sensor ID Low

MODBUS_HR1 = 0x0000
MODBUS_HR2 = 0x0001
MODBUS_HR32 = 0x001F

S8_MASK_OUTPUT_ALARM = 0x0001 # Alarm output status (inverted due to Open Collector)
S8_MASK_OUTPUT_PWM = 0x0002 # PWM output status (=1 -> full output)

S8_CO2_BACKGROUND_CALIBRATION = 0x7C06 # CO2 Background calibration
S8_CO2_ZERO_CALIBRATION = 0x7C07       # CO2 Zero calibration

S8_MASK_CO2_BACKGROUND_CALIBRATION = 0x0020 # CO2 Background calibration performed = 1
S8_MASK_CO2_NITROGEN_CALIBRATION = 0x0040 # CO2 Nitrogen calibration performed = 1

S8_MASK_METER_FATAL_ERROR = 0x0001             # Fatal error
S8_MASK_METER_OFFSET_REGULATION_ERROR = 0x0002 # Offset regulation error
S8_MASK_METER_ALGORITHM_ERROR = 0x0004         # Algorithm error
S8_MASK_METER_OUTPUT_ERROR = 0x0008            # Output error
S8_MASK_METER_SELF_DIAG_ERROR = 0x0010         # Self diagnostics error
S8_MASK_METER_OUT_OF_RANGE = 0x0020            # Out of range
S8_MASK_METER_MEMORY_ERROR = 0x0040            # Memory error
S8_MASK_METER_ANY_ERROR = 0x007F               # Mask to detect the previous errors

CO2req = bytes([0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5])   # Request CO2-Value
ABCreq = bytes([0xFE, 0X03, 0X00, 0X1F, 0X00, 0X01, 0XA1, 0XC3])   # Request ABC-Interval in [h] 1f in Hex -> 31 dezimal
FWreq = bytes([0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03])    # Readout FirmWareversion    
ID_Hi = bytes([0xFE, 0x04, 0x00, 0x1D, 0x00, 0x01, 0xB5, 0xC3])    # Sensor ID hi    
ID_Lo = bytes([0xFE, 0x04, 0x00, 0x1E, 0x00, 0x01, 0x45, 0xC3])    # Sensor ID lo    # in Hex 071dbfe4

def ModBus16CRC(buf, len):
    crc = 0xFFFF
    for pos in range(0,len):
        crc ^= buf[pos];                # XOR byte into least sig. byte of crc
        for i in range(8, 0,-1):        # Loop over each bit
            if ((crc & 0x0001) != 0):   # If the LSB is set
                crc >>= 1               # Shift right and XOR 0xA001
                crc ^= 0xA001
            else:                       # else LSB is not set
                crc >>= 1               # Just shift right
    
    # Note, this number has low and high bytes swapped, 
    # so use it accordingly (or swap bytes)
    return crc

def createCommand(func, reg, value, dim):
    buf_msg = [0x00]*dim
    if ((func == MODBUS_FUNC_READ_HOLDING_REGISTERS or
        func == MODBUS_FUNC_READ_INPUT_REGISTERS) and value >= 1) or (func == MODBUS_FUNC_WRITE_SINGLE_REGISTER):
        buf_msg[0] = MODBUS_ANY_ADDRESS    # Address
        buf_msg[1] = func                  # Function
        buf_msg[2] = (reg >> 8) & 0x00FF   # High-input register
        buf_msg[3] = reg & 0x00FF          # Low-input register
        buf_msg[4] = (value >> 8) & 0x00FF # High-word to read or setup
        buf_msg[5] = value & 0x00FF        # Low-word to read or setup
        crc16 = ModBus16CRC(buf_msg, 6)
        buf_msg[6] = crc16 & 0x00FF
        buf_msg[7] = (crc16 >> 8) & 0x00FF
    return bytes(buf_msg)

def valueFromMessage(message):
    return ((message[3] << 8) & 0xFF00) | (message[4] & 0x00FF)

def convertValueFromMessage(message):
    return message[3] * 256 + message[4]

def checkCRC(message):
    readCRC = message[6] * 256 + message[5]
    dataCRC = ModBus16CRC(message, 5)
    return dataCRC == readCRC

def validResponseLenght(buf_msg, func, nb, len):
    if (nb == len):
        result = validResponse(buf_msg, func, nb)
    else:
        result = False
        print(f"Err: Unexpected length: {nb} instead of {len}" )
    return result

def validResponse(buf_msg, func, nb):
    if nb >= 7:
        crc16 = ModBus16CRC(buf_msg, nb - 2);
        if ((buf_msg[nb-2] == (crc16 & 0x00FF)) and (buf_msg[nb-1] == ((crc16 >> 8) & 0x00FF))):
            if (buf_msg[0] == MODBUS_ANY_ADDRESS and (buf_msg[1] == MODBUS_FUNC_READ_HOLDING_REGISTERS or buf_msg[1] == MODBUS_FUNC_READ_INPUT_REGISTERS) and buf_msg[2] == nb-5):
                result = True
            else:
                result = False
                print("Err: Unexpected response!")
        else:
            result = False
            print("Err: Checksum/length is invalid!")
    else:
        result = False
        print("Err: Invalid length!")
    return result

def getSensorTypeId(sensorref):
    message = createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR26, 0x0001, 8)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = sensorref.read(S8_MSG_RESPONSE_LEN)
    if (validResponseLenght(response, MODBUS_FUNC_READ_INPUT_REGISTERS, len(response), S8_MSG_RESPONSE_LEN)):
        sensorType = ((response[4] << 16) & 0x00FF0000)
        message = createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR27, 0x0001, 8)
        sensorref.write(message)
        time.sleep(S8_TIMEOUT)  # Wait for the response
        response = sensorref.read(S8_MSG_RESPONSE_LEN)
        if (validResponseLenght(response, MODBUS_FUNC_READ_INPUT_REGISTERS, len(response), S8_MSG_RESPONSE_LEN)):
            sensorType |= ((response[3] << 8) & 0x0000FF00) | (response[4] & 0x000000FF)
        else:
            sensorType = 0
            print("Error getting sensor type ID (low)!")
    else:
            sensorType = 0
            print("Error getting sensor type ID (high)!")
    return sensorType    

def getAcknowledgement(sensorref): 
    message = createCommand(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR1, 0x0001, 8)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = sensorref.read(S8_MSG_RESPONSE_LEN)
    if len(response) == S8_MSG_RESPONSE_LEN:
        flags = valueFromMessage(response)
    else:
        flags = 0
        errMessage = "Error getting acknowledgement flags!"
        print(errMessage)
    return flags

def isBaseLineCalibrationDone(sensorref):
    if (getAcknowledgement(sensorref) & S8_MASK_CO2_BACKGROUND_CALIBRATION):
        return True
    return False

def sendSpecialCommand(sensorref, cmd):
    message = createRequest(MODBUS_FUNC_WRITE_SINGLE_REGISTER, MODBUS_HR2, cmd, 8)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = ser.read(S8_MSG_RESPONSE_LEN_CAL)
    if len(response) == S8_MSG_RESPONSE_LEN_CAL:
        result = True
        msg = "Successful setting user special command"
    else:
        result = False
        msg = "Error in setting user special command!\n"+response
    print(msg)
    return result

def getOutputStatus(sensorref):
    message = createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR3, 0x0001, 8);
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)
    response = sensorref.read(7)
    if (validResponseLenght(message, MODBUS_FUNC_READ_INPUT_REGISTERS, len(message), 7)):
        status = valueFromMessage(response)
    else:
        status = 0
        print("Error getting output status!")
        print(response)
    return status

def getAlarmStatus(sensorref):
    # Ask alarm status
    message = createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR2, 0x0001, 8);
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)
    response = sensorref.read(7)
    if validResponseLenght(message, MODBUS_FUNC_READ_INPUT_REGISTERS, len(message), 7):
        status = valueFromMessage(response)
    else:
        status = 0
        print("Error getting alarm status!")
        print(response)
    return status

def getAbcPeriod(sensorref):
    return getCalibPeriodABC(sensorref)

def getCalibPeriodABC(sensorref):
    #Ask ABC period
    message = createCommand(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR32, 0x0001, 8)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = sensorref.read(S8_MSG_RESPONSE_LEN_CAL)
    # Check response and get data
    if (validResponseLenght(response, MODBUS_FUNC_READ_HOLDING_REGISTERS, len(response), 7)):
        period = valueFromMessage(response)
#        print(f"ABC period: {period} hour")
    else:
        period = 0
        print(response)
        print("ABCreq: Invalid response from sensor")
        print("Error getting ABC period!");
    return period

# Manual calibration sensor, must be place sensor at clean environment
# for 5 minute before calib
def manualCalib(sensorref):
    result = clearAcknowledgement(sensorref)
    if result:
        result = sendSpecialCommand(sensorref, S8_CO2_BACKGROUND_CALIBRATION)
        if result:
            print("Manual calibration in background has started")
        else:
            print("Error starting manual calibration!")
        return result

def get_info(sensorref):
    message =  createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR30, 0x0001, 8)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = sensorref.read(S8_MSG_RESPONSE_LEN)  # Read 7 bytes from the sensor
    if (validResponseLenght(response, MODBUS_FUNC_READ_HOLDING_REGISTERS, len(response), S8_MSG_RESPONSE_LEN)):
        sensorID = ((response[3] << 24) & 0xFF000000) | ((response[4] << 16) & 0x00FF0000)
        message =  createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR31, 0x0001, 8)
        sensorref.write(message)
        time.sleep(S8_TIMEOUT)  # Wait for the response
        response = sensorref.read(S8_MSG_RESPONSE_LEN)  # Read 7 bytes from the sensor
        if (validResponseLenght(response, MODBUS_FUNC_READ_HOLDING_REGISTERS, len(response), S8_MSG_RESPONSE_LEN)):
            sensorID |= ((response[3] << 8) & 0x0000FF00) | (response[4] & 0x000000FF)
        else:
            sensorID = 0
            print("Error getting sensor ID (low)!")
    else:
        sensorID = 0
        print("Error getting sensor ID (high)!")
    return sensorID    

# def getPWM(sensorref):
#     """
#     Reads the PWM signal from the Senseair S8 CO2 sensor and returns CO2 concentration in ppm.
    
#     :param gpio_pin: reference to the sensor
#     :return: CO2 concentration in ppm.
#     """    

#     # Read PWM pulse widths
#     high_time = pi.get_PWM_high(gpio_pin)  # Time the signal is HIGH
#     low_time = pi.get_PWM_low(gpio_pin)    # Time the signal is LOW

#     if high_time is None or low_time is None:
#         raise RuntimeError("Failed to read PWM signal from sensor")

#     period = high_time + low_time  # Total PWM period
#     duty_cycle = high_time / period  # Duty cycle ratio

#     # Convert PWM duty cycle to CO2 concentration using the S8 formula
#     co2_ppm = 5000 * (duty_cycle - 0.04) / 0.32

#     return round(co2_ppm, 2)    

def getOutputPWM(sensorref):
    message = createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR22, 0x0001, 8)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = sensorref.read(7)  # Read 7 bytes from the sensorref
#    if (validResponseLenght(response, MODBUS_FUNC_READ_INPUT_REGISTERS, len(response), 7)):
    pwm = valueFromMessage(response)
    print(f"PWM output (raw) = {pwm}")
    pwm_ppm = (pwm / 16383.0) * 2000.0
    print(f"PWM output (to ppm, normal version) = {pwm_ppm} ppm")
#    else:
#        pwm = 0
#        print("Error getting PWM output!")
#        print(response)
    return pwm

def getCO2(sensorref):
    message = createCommand(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR4, 0x0001, 8)
#    print("CO2", message)
    sensorref.write(message)
    time.sleep(S8_TIMEOUT)  # Wait for the response
    response = sensorref.read(7)  # Read 7 bytes from the sensorref
    if (validResponseLenght(response, MODBUS_FUNC_READ_INPUT_REGISTERS, len(response), 7)):
        co2 = response[3] * 256 + response[4]
#        print(f"CO2 Concentration: {co2} ppm")
    else:
        co2 = 0
        print("Error getting CO2 output!")
        print(response)
    return co2


port = sys.argv[1]
try:
    with serial.Serial(port, S8_BAUDRATE, timeout=1) as ser:
        sid = get_info(ser)
        if sid > 0:
            print(f"Sensor ID: {sid}")
        stype = getSensorTypeId(ser)
        print(f"Sensor type: {stype}")
        period = getCalibPeriodABC(ser)
        if period > 0:
            print(f"ABC period: {period} hour")
        res = isBaseLineCalibrationDone(ser)
        print(f"Baseline calibration done: {res}")
        getOutputPWM(ser)
        while(True):
            co2 = getCO2(ser)
            if co2 > 0 and co2 <= 500:
                current_time = datetime.now().strftime("%H:%M:%S")
                print(f"{current_time} CO2: {co2} ppm")
            time.sleep(SENSOR_CO2_UPDATE_INTERVAL*10)

except serial.SerialException as e:
    print(f"Error: {e}")

