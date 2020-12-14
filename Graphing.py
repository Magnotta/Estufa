# -*- coding: utf-8 -*-
"""
Created on Wed Nov 13 11:43:41 2019

@author: Anndronius
"""

import serial
from datetime import date, datetime

def now_minutes():
    now = datetime.now().strftime("%H:%M:%S")
    now_nos = now.split(sep=':')
    now_min = 0.0
    multi = 60
    for no in now_nos:
        now_min += float(no)*multi
        multi /= 60
    return now_min

def main():
    levels = ['0','20','40','60','80']
    reads = 300

    s = serial.Serial('COM3', 9600, timeout=200)

    msg = s.read(5)

    if msg.decode() == 'begin':
        
        print("Experiment beginning at " + datetime.now().strftime("%H:%M:%S"))
        print("Experiment will last {} hours.".format(reads*len(levels)/240))
        filename = date.today().strftime("results%d_%m_%y") + ".txt"
        
        for level in levels:
            s.write(('t' + level + '\n').encode())
            
            for _ in range(reads):
                
                out_data = []
            
                msg = s.readline().decode()
                in_data = msg.split(' ')
                        
                out_data.append(now_minutes())
                
                out_data.append(float(level))
                        
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
                
                with open(filename, 'a') as f:
                    f.write('{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(*out_data))
                
            

    else:
        
        s.close()
        print('begin not received')
        exit()

    s.write('t0\n'.encode())
    s.close()

if __name__=="__main__":
    main()