# pipeswimrobot

#function of the robot:Every day, pipeline leakage causes a large amount of water (or oil, gas,etc.) lost. The old way to detect the pipeline leakage always consume a lot of times and human resources. It is in need to have an new leakage detection	way to offer help. SwimBot is designed for low-cost, constantly-running leakage detection.
#
#Design:

location detecting:
Using the IMU built in the Arduino 101 board, which include 3-axis gyroscope and 3-axis accelerometer, to detect the position.
Writing data as a format of the number, time, microphone, 3-axis gyroscope and 3-axis accelerometer into sd card.
Using outside Matlab function to analyzing the data we load in SD card to get the position of leaking. and also use machine learning to simulate the size of leaking

Microphone 
Maxim MAX4466 is an op-amp specifically designed for a delicate task which can recorded 20-20KHZ electret sounds. This microphone is very suitable for detecting varieties size of leaking. 

M100 Brushless Motor with propeller ($82.00)
This motor is a rugged brushless motor for use in the ocean and in extreme environments. This propeller with M100 motor can generates about 3.2 lb of forward thrust and 2.6 ln of  reverse thrust.

Speed controller 
This controller is based on the BLHeli_S ESC design, providing an speed control and voltage adjustment for Motors.

Outfit
Using 3D printer to print a special outer casing or shell for this swimming robot. The whole device is designed like a bullet so it can reduce the force of water. There is a gap surrounding the chips so it can isolate water from chips
