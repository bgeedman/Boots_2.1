#!/usr/bin/python

import argparse
import pygame
import socket
import struct
import sys
from enum import IntEnum
from numpy import interp
from pygame.locals import *


class Buttons(IntEnum):
    TRIGGER = 0
    THUMB = 1
    AUX11 = 11

class Axis(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2
    AUX = 3



'''
    This will be the main loop when running in keyboard mode. It currently does
    not work because pygame requires a screen to capture the keyboard events. I
    will need to research and design what kind of UI to be presented to the user.
'''
def run_keyboard(server, clock):
    print("Keyboard mode is currently not working because it requires a pygame.screen")




'''
    This will be the main loop when running in joystick mode. It currently only
    works on the command line mode, but when I decide to add the keyboard part,
    I will port it to the joystick as well. Also, there may be a better way of
    setting this up in a class as to make the event loop cleaner.

    Quick overview, the joystick trigger button engages stretch mode when pressed
    and when it is released exits stretch mode. Stretch mode is just a way to show
    off the Inverse Kinematics and really serves no other purpose.  Other buttons
    have no use either.

    To manuver the robot, just move the joystick in normal mode.  The pitch and roll
    axes are used to calculate a vector on which to walk.  The yaw axis is used to
    send the turn command.  To turn, you must be in neutral.
'''
def run_joystick(server, name, clock):
    print("Running in joystick mode")
    joystick = None
    stretch_mode = False
    joystick_count = pygame.joystick.get_count()
    if (joystick_count == 0):
        print("No joysticks detected.")
        print("Check that one is plugged in or run with --keyboard option")
        return
    elif (joystick_count == 1):
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    else:
        for i in range(joystick_count):
            tmp_joystick = pygame.joystick.Joystick(i)
            tmp_joystick_name = tmp_joystick.get_name()
            if (name == None):
                print("Available joysticks")
                print("\t- {}".format(tmp_joystick_name))
            else:
                if (tmp_joystick_name == name):
                    joystick = tmp_joystick
                    joystick.init()
                    break
    if (joystick == None):
        print("Failed to locate a valid joystick")
        return
    print("Using joystick: {}".format(joystick.get_name()))
    print("Available buttons: {}".format(joystick.get_numbuttons()))
    print("Available axis: {}".format(joystick.get_numaxes()))
    print("Available hats: {}".format(joystick.get_numhats()))
    print("------------------------------------------------------")
    done = False

    stretch_params= {'pitch':0.0, 'roll':0.0, 'yaw':0.0, 'delta_x':0.0, 'delta_y':0.0}

    pygame.time.set_timer(USEREVENT + 1, 1000)

    while (done != True):
        for event in pygame.event.get():
            if (event.type == pygame.QUIT):
                done = True

            '''
                We have a button pressed, we must handle each button on it's own
                There is currently only two buttons we care about. The trigger,
                which will enter stretch mode when pressed, and the auxilary 11,
                which will quit the program.
            '''
            if (event.type == pygame.JOYBUTTONDOWN):
                if (event.button == Buttons.TRIGGER):
                    print("Trigger button pressed")
                    stretch_mode = True
                elif (event.button == Buttons.AUX11):
                    done = True
                else:
                    print("Unhandled button pressed")

            '''
                We have a button released, and like the button press, handle each
                button on it's own.  Trigger release will indicate exiting stretch
                mode. Should leaving stretch mode recenter the body?
            '''
            if (event.type == pygame.JOYBUTTONUP):
                if (event.button == Buttons.TRIGGER):
                    print("Trigger button released")
                    stretch_mode = False
                else:
                    print("Unhandled button released")

            '''
                We have an axis motion event. This indicates one of the joystick
                axes have changed position. Can become quite jittery, and need
                to look into having some kind of jitter filter.

                If we are in stretch mode, the PITCH, ROLL, and YAW axes will
                correspond to rotating the body up to a max/min of 30 degrees.
                The aux axis currently has no usage.

                If we are in normal mode, th PITCH and ROLL will be used to
                calculate a vector on which the robot should walk. And the YAW
                axis will be used to send TURN commands.
            '''
            if (event.type == pygame.JOYAXISMOTION):
                if (stretch_mode):
                    if (event.axis == Axis.ROLL):
                        stretch_params['roll'] = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                    if (event.axis == Axis.PITCH):
                        stretch_params['pitch'] = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                    if (event.axis == Axis.YAW):
                        stretch_params['yaw'] = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                else:
                    ''' This is where we would create the walk vector'''
                    pass

            '''
                The hat on top of joystick has moved.  If in stretch mode, this
                will perform a translation. Normal mode will have no affect at
                this time.

            '''
            if (event.type == pygame.JOYHATMOTION):
                if (stretch_mode):
                    stretch_params['delta_x'] = event.value[0]
                    stretch_params['delta_y'] = event.value[1]

            '''
                This is a timer event set to trigger 10 times every second. The
                idea being that we won't spam the server with commands everytime
                the joystick moves. Still considering if this is a valid solution
                or if we should just accept the XTREME traffic.
            '''
            if (event.type == USEREVENT + 1):
                if (stretch_mode):
                    #print("Sending rotation: {}".format(stretch_params))
                    send_data(None, stretch_params)




def send_data(sock, data):
    buf = struct.pack('!fffbb', data['roll'], data['pitch'], data['yaw'], data['delta_x'], data['delta_y'])
    print len(buf)
    print buf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joystick', help="name of joystick", type=str)
    parser.add_argument('--keyboard', help="run keyboard interface", action="store_true")
    parser.add_argument('--server', help='server IP address to connect', type=str)
    parser.add_argument('--verbose', help="increase output verbosity", action="store_true")
    args = parser.parse_args()
    pygame.init()
    clock = pygame.time.Clock()
    if (args.keyboard):
        run_keyboard(args.server, clock)
    else:
        run_joystick(args.server, args.joystick, clock)
    pygame.quit()



if __name__ == "__main__":
    main()
