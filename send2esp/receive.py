import numpy as np
import serial
import socket
import pandas as pd
import csv

def setupSerial():
  ser = serial.Serial()
  ser.baudrate = 19200
  # ser.port = 'COM9'
  ser.port = '/dev/ttyUSB0'

if __name__ == "__main__":

  # create a socket object to send/receive via udp
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
    host_ip = '0.0.0.0'
    host_port = 54321
    sock.bind((host_ip, host_port))
    print('host_ip:port = ', host_ip, ':', host_port, '\n')

    with open('output.csv', mode='w', newline='') as file:
      writer = csv.writer(file)
      while True:
        data, addr = sock.recvfrom(256)

        # process the received data
        data_str = data.decode()
        # Split the string into a list of substrings
        data_list = data_str.split(',')
        # Convert each substring to a float and store them in a list
        data_float = [float(x) for x in data_list]
        
        # write to a csv file
        writer.writerow(data_float)
        print('data: ', data, '\n')
        print('addr: ', addr, '\n')

        # send signal to send.py
        sock.sendto(b'finished logging', (host_ip, 54323))
  
  # TODO: if the message is the calculation result, log it;
  # if it is an end signal, jump out of the loop and write 
  # the result log to a csv file
