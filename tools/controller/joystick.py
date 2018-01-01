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
import binascii


class Buttons(IntEnum):
    TRIGGER = 0
    THUMB = 1
    AUX11 = 11


class Axis(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2
    AUX = 3


class Cmd(IntEnum):
    STOP = 0
    STAND = 1
    WALK = 2
    TURN = 3
    STRETCH = 4
    PARK = 5
    STATUS = 6

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
    header = commands_pb2.Header()
    header.cmd = Cmd.STOP
    message = None

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
                if (event.button == Buttons.TRIGGER):
                    stretch_mode = True
                    header.cmd = Cmd.STRETCH
                    message = commands_pb2.Stretch()
                    message.roll = 0.0
                    message.pitch = 0.0
                    message.yaw = 0.0
                    message.delta_x = 0
                    message.delta_y = 0
                elif (event.button == Buttons.AUX11):
                    done = True
                else:
                    logging.warning("Unhandled button down event")


            '''
                We have a button released, and like the button press, handle each
                button on it's own.  Trigger release will indicate exiting stretch
                mode. Should leaving stretch mode recenter the body?
            '''
            if (event.type == pygame.JOYBUTTONUP):
                if (event.button == Buttons.TRIGGER):
                    header.cmd = Cmd.STOP
                    message = None
                    stretch_mode = False
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
                        message.roll = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                    if (event.axis == Axis.PITCH):
                        message.pitch = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                    if (event.axis == Axis.YAW):
                        message.yaw = interp(event.value, [-1.0, 1.0], [-30.0, 30.0])
                else:
                    ''' This is where we would create the walk vector, or the turn event'''
                    if (event.axis == Axis.PITCH and header.cmd != Cmd.TURN):
                        if (event.value > 0.5 or event.value < -0.5):
                            header.cmd = Cmd.WALK
                            if (not isinstance(message, commands_pb2.Walk)):
                                message = commands_pb2.Walk()
                            if (event.value > 0.5):
                                message.dir = Dir.BACKWARD
                            else:
                                message.dir = Dir.FORWARD
                        else:
                            header.cmd = Cmd.STOP
                            message = None
                    if (event.axis == Axis.YAW and header.cmd != Cmd.WALK):
                        if (event.value > 0.5 or event.value < -0.5):
                            header.cmd = Cmd.TURN
                            if (not isinstance(message, commands_pb2.Turn)):
                                message = commands_pb2.Turn()
                            if (event.value > 0.5):
                                message.dir = Dir.RIGHT
                            else:
                                message.dir = Dir.LEFT
                        else:
                            header.cmd = Cmd.STOP
                            message = None

            '''
                The hat on top of joystick has moved.  If in stretch mode, this
                will perform a translation. Normal mode will have no affect at
                this time.

            '''
            if (event.type == pygame.JOYHATMOTION):
                if (stretch_mode):
                    message.delta_x = event.value[0]
                    message.delta_y = event.value[1]


            '''
                This is a timer event set to trigger 10 times every second. The
                idea being that we won't spam the server with commands everytime
                the joystick moves. Still considering if this is a valid solution
                or if we should just accept the XTREME traffic.
            '''
            if (event.type == USEREVENT + 1):
                send_message(sock, header, message)


    if (sock != None):
        logging.info("Closing connection to server")
        sock.close()
    pygame.quit()






'''
    Send out a new message to the server. The header contains the command type
    and the length of the proceeding message. If no message is attached, the
    length will be 0 and we only send the header.
'''
def send_message(sock, header, message):
    logging.debug("Sending new message")
    if (sock):
        message_len = 0
        if (message):
            message_data = message.SerializeToString()
            message_len = len(message_data)
        header.len = message_len
        header_data = header.SerializeToString()
        header_len = len(header_data)
        logging.debug("Header length: {}".format(header_len))
        logging.debug("Header data: 0x{}".format(binascii.hexlify(header_data)))
        sock.send(struct.pack('!i', header_len))
        sock.send(header_data)
        if (message):
            logging.debug("Message data: 0x{}".format(binascii.hexlify(message_data)))
            sock.send(message_data)
    else:
        logging.debug("Header: {}".format(header))
        logging.debug("Message: {}".format(message))



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
