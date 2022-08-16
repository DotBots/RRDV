import csv
import serial
import sys
import time

import OpenHdlc
import struct

RANGE_ALL = [0x1F]
RANGE_1_2 = [0x03]
RANGE_1_3 = [0x05]
RANGE_1_4 = [0x09]
RANGE_1_5 = [0x11]
RANGE_2_3 = [0x06]
RANGE_2_4 = [0x0A]
RANGE_2_5 = [0x12]
RANGE_3_4 = [0x0C]
RANGE_3_5 = [0x14]
RANGE_4_5 = [0x18]


# Open com port
ser = serial.Serial('COM14', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
number_of_measurements = 10
line_count = 1

in_buf = []

trigger_data = RANGE_1_4


_hdlc       = OpenHdlc.OpenHdlc()
hdlc_data_w = _hdlc.hdlcify(trigger_data) 

hdlcBusyReceiving = False
last_rx_byte_int  =  0x00 

range_measurements_unpack = []
robot_measurements        = []
robot_ids                 = []
timestamp                 = []

while line_count <= number_of_measurements:

    ser.write(hdlc_data_w)
    
    t           = time.time()
    time_now_ms = int(t*1000)
    
    hdlcFrameReceived = False
    
    # trigger the US ranging and get the HDLC frame
    while hdlcFrameReceived == False:
        rx_byte = ser.read(1)        
               
        rx_byte_int = int.from_bytes(rx_byte, "big")
        
        if rx_byte_int != _hdlc.HDLC_FLAG and last_rx_byte_int == _hdlc.HDLC_FLAG and hdlcBusyReceiving == False: 
            in_buf.append(_hdlc.HDLC_FLAG)  
            in_buf.append(rx_byte_int)            
            hdlcBusyReceiving = True                     
        elif rx_byte_int != _hdlc.HDLC_FLAG and hdlcBusyReceiving == True:
            in_buf.append(rx_byte_int)
        elif rx_byte_int == _hdlc.HDLC_FLAG and hdlcBusyReceiving == True:
            in_buf.append(_hdlc.HDLC_FLAG)
            hdlcBusyReceiving = False
            hdlcFrameReceived = True   
                                     
        last_rx_byte_int = rx_byte_int

    # unpack the measurements of each robot
    dehdlc_data       = _hdlc.dehdlcify(in_buf)          
    for i in range(0, len(dehdlc_data), 5):
        robot_ids.append(dehdlc_data[i])
        range_measurements_unpack = dehdlc_data[i+1:i+5]             
        range_us          = int.from_bytes(range_measurements_unpack, byteorder="little")    
        range_cm          = range_us/58        
        robot_measurements.append(range_cm)
        timestamp.append(time_now_ms)
              
   
    print(f"Measurement {line_count} done")

    # Increment the line count, and stop the loop
    # once we have 100 lines
    line_count += 1
    in_buf      = []
    
rows = zip(robot_ids, robot_measurements, timestamp)
    
with open("datafile_tx.csv", "w", newline='') as new_file:
    csv_writer = csv.writer(new_file)
    for row in rows:
        csv_writer.writerow(row)