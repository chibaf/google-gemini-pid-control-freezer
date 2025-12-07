class arduino:
#
  def __init__(self):
    #read serials#    from read_m5b_class import m5logger
    import serial
    self.ser0=serial.Serial("/dev/ttyACM0",19200)
    self.ser1=serial.Serial("/dev/ttyACM1",19200)
#   
  def read(self):
    i=0
    while True:
      i=i+1
      line01 = self.ser0.readline()
      line02 = self.ser1.readline()
      try:
        line01s=line01.strip().decode('utf-8')
        line02s=line02.strip().decode('utf-8')
      except UnicodeDecodeError:
        continue
      data1s=line01s.split(",")
      data2s=line02s.split(",")
      if len(data1s)!=9 or len(data2s)!=9:
        continue
      data1=[]
      data2=[]
      for i in range(0,8):
        data1=data1+[data1s[i+1]]
        data2=data2+[data2s[i+1]]
      try:
        data1s = [float(val) for val in data1]
        data2s = [float(val) for val in data2]
      except Exception as e:
        continue
      break
    if data1s[0]=='A2':
      array=data2s+data1s
    else:
      array=data1s+data2s
    return array
#    
  def close(self):
    self.ser0.close()   
    self.ser1.close()   
#
class thread_ssr:
  def __init__(self):   # initial action
    return
  def thread(self,it,q): # class body
    a=q.get()   # get Tc temp
#
    if time.time()-a[10]>=1800.: # freezer on/off
      a[10]=time.time()
      ssr18=0
    elif 0<time.time()-a[10]<=1500.:
      if a[0]<-20.0:   # freezer off -20と極端にしてみた {250865}
        ssr18=0
      else:
        ssr18=1 # 1->on, 0->off
    else:
      ssr18=0     
    # トータル 80000秒周期（1day=86400）での温度閾値切り替えロジックにした {250826}
    # 過冷却になっている可能性が高いので、-0.5 （薄い氷にしたかった）から、-2.0にしてみる {250628}
    time_elapsed = time.time() - a[11]
    if time_elapsed >= 80000.:
      a[11] = time.time()
      temp = -2.0  # 周期リセット時は-0.5に設定
    elif time_elapsed <= 70000.:
      temp = -2.0
    else:
      temp = 0.5
    av=sum(a[1:10])/9. # heater
    if av>temp:  # threshold of ssr switching  # 0.5 -> 0.0 24/JUl/2025 # 0.0->0.5 20250725
      ssr1=0   # ssr off
    else:
      ssr1=1   # ssr on
    print([ssr1,ssr18,a,f"time_elapsed:{time_elapsed:.1f}",f"temp:{temp}",f"av:{av:.2f}"])
    q.put([ssr1,ssr18,a[10],a[11]])   # set ssr value to queu
    return

### ### ###
import serial
import time
from datetime import date
import matplotlib.pyplot as plt
import threading
import queue  # library for queu operation
import RPi.GPIO as GPIO
#
#import arduino
#
ard=arduino()
while 1:
  try:
    print(ard.read())
  except KeyboardInterrupt:
    ard.close()
    exit()
