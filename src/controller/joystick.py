#!/usr/bin/python

import argparse
import logging
import pygame
import socket
import struct
import sys
from enum import IntEnum
from numpy import interp
from pygame.locals import *
import commands_pb2


class Buttons(IntEnum):
    TRIGGER = 0
    THUMB = 1
    AUX06 = 6
    AUX11 = 11


class Axis(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2
    AUX = 3


class Cmds(IntEnum):
    STOP = 0
    STAND = 1
    WALK = 2
    TURN = 3
    STRETCH = 4
    PARK = 5
    QUIT = 6


class Dir(IntEnum):
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3


'''
    This will be the main loop when running in keyboard mode. It currently does
    not work because pygame requires a screen to capture the keyboard events. I
    will need to research and design what kind of UI to be presented to the user.
'''
def run_keyboard(server):
    logging.error("Unimplemented function called")




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
def run_joystick(server, name):
    logging.info("Running joystick mode")
    pygame.init()
    joystick = None
    stretch_mode = False
    sock = None
    port = 15000

    command = commands_pb2.Command()
    command.cmd = Cmds.STOP

    joystick_count = pygame.joystick.get_count()
    if (joystick_count == 0):
        logging.error("No joystick detected")
        logging.error("Check that one is plugged in or run with --keyboard")
        return
    elif (joystick_count == 1):
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    else:
        logging.error("To many joysticks detected")
        return

    if (joystick == None):
        loggign.critical("Failed to locate a valid joystick")
        return
    done = False

    pygame.time.set_timer(USEREVENT + 1, 1000)

    if (server != None):
        logging.info("Connecting to server {}".format(server))
        sock = socket.socket()
        sock.connect((server, port))
        if (not sock):
            logging.error("Failed to connect to {}:{}".format(server, port))
            return


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
                if (event.button == Buttons.TRIGGER and command.cmd == Cmds.STOP):
                    stretch_mode = True
                    command.cmd = Cmds.STRETCH
                elif (event.button == Buttons.AUX11):
                    command.cmd = Cmds.QUIT
                    done = True
                elif (event.button == Buttons.AUX06):
                    command.cmd = Cmds.STAND
                else:
                    logging.warning("Unhandled button down event")


            '''
                We have a button released, and like the button press, handle each
                button on it's own.  Trigger release will indicate exiting stretch
                mode. Should leaving stretch mode recenter the body?
            '''
            if (event.type == pygame.JOYBUTTONUP):
                if (event.button == Buttons.TRIGGER or
                    event.button == Buttons.AUX06):
                    stretch_mode = False
                    command.cmd = Cmds.STOP
                    command.roll = 0.0
                    command.pitch = 0.0
                    command.yaw = 0.0
                    command.delta_x = 0
                    command.delta_y = 0
                else:
                    logging.warning("Unhandled button up event")


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
                        command.roll = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                    if (event.axis == Axis.PITCH):
                        command.pitch = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                    if (event.axis == Axis.YAW):
                        command.yaw = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                else:
                    ''' This is where we would create the walk vector, or the turn event '''
                    if (event.axis == Axis.PITCH and command.cmd != Cmds.TURN):
                        if (event.value > 0.5):
                            command.cmd = Cmds.WALK
                            command.dir = Dir.BACKWARD
                        elif (event.value < -0.5) :
                            command.cmd = Cmds.WALK
                            command.dir = Dir.FORWARD
                        else:
                            command.cmd = Cmds.STOP
                    if (event.axis == Axis.YAW and command.cmd != Cmds.WALK):
                        if (event.value > 0.5):
                            command.cmd = Cmds.TURN
                            command.dir = Dir.RIGHT
                        elif (event.value < -0.5):
                            command.cmd = Cmds.TURN
                            command.dir = Dir.LEFT
                        else:
                            command.cmd = Cmds.STOP

            '''
                The hat on top of joystick has moved.  If in stretch mode, this
                will perform a translation. Normal mode will have no affect at
                this time.

            '''
            if (event.type == pygame.JOYHATMOTION):
                if (stretch_mode):
                    command.delta_x = event.value[0]
                    command.delta_y = event.value[1]

            '''
                This is a timer event set to trigger 10 times every second. The
                idea being that we won't spam the server with commands everytime
                the joystick moves. Still considering if this is a valid solution
                or if we should just accept the XTREME traffic.
            '''
            if (event.type == USEREVENT + 1):
                send_command(sock, command)

    # Send the one final command
    send_command(sock, command)

    if (sock != None):
        logging.info("Closing connection to server")
        sock.close()
    pygame.quit()




def send_command(sock, command):
    if (sock):
        cmd_data = command.SerializeToString()
        cmd_len = len(cmd_data)
        logging.debug("Sending len: {}".format(cmd_len))
        sock.send(struct.pack('!i',cmd_len))
        logging.debug("Sending cmd: {}".format(command))
        sock.send(cmd_data)
    else:
        logging.debug("Command:\n{}".format(command))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joystick', help="name of joystick", type=str)
    parser.add_argument('--keyboard', help="run keyboard interface", action="store_true")
    parser.add_argument('--server', help='server IP address to connect', type=str)
    args = parser.parse_args()
    logging.basicConfig(format='%(asctime)-15s [%(levelname)s]: %(message)s',
                        level=logging.DEBUG);

    if (args.keyboard):
        run_keyboard(args.server)
    else:
        run_joystick(args.server, args.joystick)



if __name__ == "__main__":
    main()
