#!/usr/bin/python

#   @file calibration.py
#   @description:
#       This file is used to configure the servos on the robot. Should only
#       need to run this tool once to determine the max's and min's.  Needs
#       to be run sudo to open the TTY.
#
#   @written Bryan Geedman
#
#   @TODO:
#       - Generate a config file to be consumed by main runner



from Tkinter import *
import serial

FRONT_LEFT_SHOULDER = 27
FRONT_LEFT_FEMUR = 28
FRONT_LEFT_TIBIA = 29

FRONT_RIGHT_SHOULDER = 11
FRONT_RIGHT_FEMUR = 12
FRONT_RIGHT_TIBIA = 13

BACK_LEFT_SHOULDER = 20
BACK_LEFT_FEMUR = 19
BACK_LEFT_TIBIA = 18

BACK_RIGHT_SHOULDER = 4
BACK_RIGHT_FEMUR = 3
BACK_RIGHT_TIBIA = 2






ser = serial.Serial("/dev/ttyUSB0", 115200)

# Front right leg
def front_right_shoulder_handler(value):
    string = "#{0} P{1}\r\n".format(FRONT_RIGHT_SHOULDER, value)
    print string
    ser.write(string)

def front_right_femur_handler(value):
    string = "#{0} P{1}\r\n".format(FRONT_RIGHT_FEMUR, value)
    print string
    ser.write(string)

def front_right_tibia_handler(value):
    string = "#{0} P{1}\r\n".format(FRONT_RIGHT_TIBIA, value)
    print string
    ser.write(string)



# Front left leg
def front_left_shoulder_handler(value):
    string = "#{0} P{1}\r\n".format(FRONT_LEFT_SHOULDER, value)
    print string
    ser.write(string)

def front_left_femur_handler(value):
    string = "#{0} P{1}\r\n".format(FRONT_LEFT_FEMUR, value)
    print string
    ser.write(string)

def front_left_tibia_handler(value):
    string = "#{0} P{1}\r\n".format(FRONT_LEFT_TIBIA, value)
    print string
    ser.write(string)



# Back right leg
def back_right_shoulder_handler(value):
    string = "#{0} P{1}\r\n".format(BACK_RIGHT_SHOULDER, value)
    print string
    ser.write(string)

def back_right_femur_handler(value):
    string = "#{0} P{1}\r\n".format(BACK_RIGHT_FEMUR, value)
    print string
    ser.write(string)

def back_right_tibia_handler(value):
    string = "#{0} P{1}\r\n".format(BACK_RIGHT_TIBIA, value)
    print string
    ser.write(string)



# Back left leg
def back_left_shoulder_handler(value):
    string = "#{0} P{1}\r\n".format(BACK_LEFT_SHOULDER, value)
    print string
    ser.write(string)

def back_left_femur_handler(value):
    string = "#{0} P{1}\r\n".format(BACK_LEFT_FEMUR, value)
    print string
    ser.write(string)

def back_left_tibia_handler(value):
    string = "#{0} P{1}\r\n".format(BACK_LEFT_TIBIA, value)
    print string
    ser.write(string)


master = Tk()

s1 = Scale(master, from_=1090, to=2120, orient=HORIZONTAL, tickinterval=250,\
        command=front_left_shoulder_handler, length=600, label="Front Left Shoulder")
s1.set(1090);
s1.pack()

f1 = Scale(master, from_=650, to=2500, orient=HORIZONTAL, tickinterval=250,\
        command=front_left_femur_handler, length=600, label="Front Left Femur")
f1.set(2500);
f1.pack()

t1 = Scale(master, from_=740, to=2275, orient=HORIZONTAL, tickinterval=250,\
        command=front_left_tibia_handler, length=600, label="Front Left Tibia")
t1.set(2275);
t1.pack()

s = Scale(master, from_=1140, to=2030, orient=HORIZONTAL, tickinterval=250,\
        command=front_right_shoulder_handler, length=600, label="Front Right Shoulder")
s.set(2030);
s.pack()

f = Scale(master, from_=710, to=2500, orient=HORIZONTAL, tickinterval=250,\
        command=front_right_femur_handler, length=600, label="Front Right Femur")
f.set(710);
f.pack()

t = Scale(master, from_=850, to=2430, orient=HORIZONTAL, tickinterval=250,\
        command=front_right_tibia_handler, length=600, label="Front Right Tibia")
t.set(850);
t.pack()


s3 = Scale(master, from_ = 1040, to=1975, orient=HORIZONTAL, tickinterval=250,\
        command=back_left_shoulder_handler, length=600, label="Back Left Shoulder")
s3.set(1975);
s3.pack()

f3 = Scale(master, from_=650, to=2465, orient=HORIZONTAL, tickinterval=250,\
        command=back_left_femur_handler, length=600, label="Back Left Femur")
f3.set(650);
f3.pack()

t3 = Scale(master, from_=815, to=2400, orient=HORIZONTAL, tickinterval=250,\
        command=back_left_tibia_handler, length=600, label="Back Left Tibia")
t3.set(815);
t3.pack()




s2 = Scale(master, from_=1115, to=1980, orient=HORIZONTAL, tickinterval=250,\
        command=back_right_shoulder_handler, length=600, label="Back Right Shoulder")
s2.set(1115);
s2.pack()

f2 = Scale(master, from_=635, to=2500, orient=HORIZONTAL, tickinterval=250,\
        command=back_right_femur_handler, length=600, label="Back Right Femur")
f2.set(2500);
f2.pack()

t2 = Scale(master, from_=730, to=2215, orient=HORIZONTAL, tickinterval=250,\
        command=back_right_tibia_handler, length=600, label="Back Right Tibia")
t2.set(2215);
t2.pack()






mainloop()
