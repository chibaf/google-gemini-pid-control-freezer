class read_m5:
#
  def __init__(self):
    #read serials#    from read_m5b_class import m5logger
    self.ser0=serial.Serial("/dev/ttyUSB0",115200)
    self.ser1=serial.Serial("/dev/ttyUSB1",115200)
#   
  def reads(self):
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
      if len(data1s)!=12 or len(data2s)!=12:
        continue
      if data1s[0]=="03":
        data=[data1s[2],data1s[3]]
      else:
        data=[data2s[2],data2s[3]]
      try:
        data1 = [float(val) for val in data]
      except Exception as e:
        continue
      break
#    print(data1)
    return data1
#    
  def close(self):
    self.ser0.close()   

import serial
import time
from datetime import date
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
#
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
#
current_time = time.strftime("_H%H_M%M_S%S", time.localtime())
fn = "LR5-SSR_" + str(date.today()) + current_time + ".csv"
f=open(fn, 'w', encoding="utf-8")
start = time.time()
#
GPIO.setup(18, GPIO.OUT)  # freezer: ssr18
#
logger = read_m5()
plt.figure(100)
y1=[0]*100
y2=[0]*100
x = range(0, 100, 1)
plt.ylim(-25,10)
#
#GPIO.output(15, 1)  # pump always runs
#
stime=time.time()
while True:
  try:
    # output data
    if time.time()-stime>1800.:
      stime=time.time()
    temp=logger.reads()
    if time.time()-stime<=1500.:
      if temp[0]>-20.0:
        GPIO.output(18,1)
        ssr18=1
      else:
        GPIO.output(18,0)
        ssr18=0
    else:
      GPIO.output(18,0)
      ssr18=0
    #
    st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
    ss = str(time.time() - int(time.time()))
    sss=str(round(time.time()-start, 2))
    row=st + ss[1:5] + "," + sss + ","
    row=row+str(temp[0])+","+str(temp[1])+","+str(ssr18)+"\n"
    f.write(row)
    print(row)
# plotting
    title=str(round(time.time()-stime,2))+',temp1='+str(round(temp[0],2))+',temp2='+str(round(temp[1],2))+','+"ssr18="+str(ssr18)
    plt.clf()
    y1.pop(-1)
    y1.insert(0,temp[0])
    y2.pop(-1)
    y2.insert(0,temp[1])
    plt.title(title)
    plt.ylim(-25,20)
    plt.plot(x,y1)
    plt.plot(x,y2)
    plt.pause(0.1)
  except KeyboardInterrupt:
    print("KeyboardInterrupt:")
    GPIO.output(18,0)
    GPIO.cleanup()
    logger.close()
    f.close()
    exit()