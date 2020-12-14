# -*- coding: utf-8 -*-
"""
Created on Sun May  3 13:48:08 2020

@author: Anndronius
"""

import threading
import serial
from datetime import date, datetime
from time import sleep
        
def cmd_recv(s):
    while True:
        cmd = input() + '\n'
        s.write(cmd.encode())
        sleep(1.0)
        
            
def now_minutes():
    now = datetime.now().strftime("%H:%M:%S")
    now_nos = now.split(sep=':')
    now_min = 0.0
    multi = 60
    for no in now_nos:
        now_min += float(no)*multi
        multi /= 60
    return now_min

def routine(s):
    y = threading.Thread(target=cmd_recv, daemon=True, args=(s,))
    y.start()
    msg = s.read(5).decode()
    if msg == "begin":
        while True:
            out_data = []
            msg = s.readline().decode()
            if msg[0] == 'r':
                print(msg[1:])
            else:
                in_data = msg.split(' ')
                
                out_data.append(now_minutes())
                
                try:
                    out_data.append(float(in_data[0]))
                except ValueError:
                    out_data.append(0.0)
                    
                try:
                    out_data.append(float(in_data[1]))
                except ValueError:
                    out_data.append(0.0)
                    
                try:
                    out_data.append(float(in_data[2]))
                except ValueError:
                    out_data.append(0.0)
                    
                try:
                    out_data.append(float(in_data[3]))
                except ValueError:
                    out_data.append(0.0)
                    
                try:
                    out_data.append(float(in_data[4]))
                except ValueError:
                    out_data.append(0.0)
                    
                filename = date.today().strftime("%d_%m_%y") + ".txt"
                    
                with open(filename, mode='a') as f:
                    f.write('{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(*out_data))
                    
                
if __name__ == "__main__":
    
    while True:
        s = serial.Serial('COM3', 9600, timeout=200)
    
        x = threading.Thread(target=routine, daemon=True, args=(s,))
        x.start()
        
        try:
            x.join()
        except Exception as e:
            print("Program crashed at {}.".format(datetime.now().strftime("%H:%M:%S")))
            print(str(e))
        finally:
            s.close()