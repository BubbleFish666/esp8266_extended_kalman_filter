import numpy as np
import serial
import socket
import pandas as pd

def setupSerial():
  ser = serial.Serial()
  ser.baudrate = 19200
  ser.port = 'COM9'

if __name__ == "__main__":

  # create a socket object to send/receive via udp
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
    host_ip = '0.0.0.0'
    host_port = 54321
    sock.bind((host_ip, host_port))
    print('host_ip:port = ', host_ip, ':', host_port, '\n')

    # create a socket object to send data to esp
    # sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # sock_send.bind(('0.0.0.0', 12345))

    esp_ip = '192.168.31.102'
    esp_port = 4210
    msg = b'Hello, ESP!'
    # for i in range(3):
      # sock.sendto(msg, (esp_ip, esp_port))

    while True:
      data, addr = sock.recvfrom(256)
      print('data: ', data, '\n')
      print('addr: ', addr, '\n')


  df = pd.read_csv('Frames_t_simpl.csv')
  # TODO: iterate over lines and proceed when conditions met

  # TODO: store the line into a string

  # TODO: send the string via UDP to ESP

  # TODO: listen to UDP port until the message from ESP is received
  
  # TODO: if the message is the calculation result, log it;
  # if it is an end signal, jump out of the loop and write 
  # the result log to a csv file

  

  # sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)