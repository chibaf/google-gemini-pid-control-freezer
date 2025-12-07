# google-pid-control-freezer

<ul>
I want to control temperature of a freezer using PID control on Raspberry Pi.<br>
<br>
time interval is 1800sec.<br>
(1) input=present freezer temperature (getting via USB serial)<br>
(2) target temperature=-20 degrees C.<br>
(3) controlling interval = 1800sec<br>
(3-1) 0<=time<=1500<br>
   controlling temperature with on/off freezer via GPIO<br>
(3-2) 1500<time<=1800<br>
   switch off freezer via GPIO<br>
(4) Next 1800sec<br>
<br>
Can you write a program for this operation by python.<br>
</ul>

<ul>
   Control a freezer’s temperature by PID method by python on Raspberry Pi<br>
(1) Target temperature: -20<br>
(2) input temperature from serial<br>
   /dev/ttyUSB0, speed=115200/sec<br>
(3) data format<br>
    tag, date, temperature x10<br>
    we use the first temperature<br>
(4) operation cycle=1800sec    <br>
   (4-1) 0~1500sec       switch on/off via GPIO=18<br>
   (4-2) 1500~1800sec<br>
     switch off via GPIO=18<br>
(5) the next operation cycle<br>
</ul>
