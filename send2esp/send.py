import socket
import numpy as np

import os

if __name__ == "__main__":

  file_path = os.path.join(os.path.dirname(__file__), 'Frames_t_simpl.csv')  
  with open(file_path, mode='r') as file:
    # load data
    measurements = np.float32(np.loadtxt(file, delimiter=',', skiprows=3).astype(float))
    # convert the north-indicating angle in degrees to radian on 4-phase plane
    measurements[:,3] = (-measurements[:,3] + 270 + 360) % 360 - 180
    measurements[:,3] = measurements[:,3] / 180 * np.pi

    start_time = 144
    # end_time = 145
    end_time = 160
    measurements = measurements[(measurements[:,0] > start_time) & (measurements[:,0] < end_time)]

    # create a socket object to send/receive via udp
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
      host_ip = '0.0.0.0'
      host_port = 54323
      sock.bind((host_ip, host_port))
      print('host_ip:port = ', host_ip, ':', host_port, '\n')
  
      # esp_ip = '192.168.12.6'
      esp_ip = '192.168.31.102'
      esp_port = 4210
      # msg = b'Hello, ESP!'

      for row in measurements:
        msg_list = row[1:].tolist()
        msg_str = ','.join(map(str, msg_list))
        msg = msg_str.encode()

        sock.sendto(msg, (esp_ip, esp_port))

        data, addr = sock.recvfrom(256)
        print('data: ', data, '\n')
        print('addr: ', addr, '\n')


