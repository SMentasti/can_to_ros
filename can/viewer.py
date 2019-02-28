# coding: utf-8
#
# Copyright (C) 2018 Kristian Sloth Lauszus.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
# Contact information
# -------------------
# Kristian Sloth Lauszus
# Web      :  http://www.lauszus.com
# e-mail   :  lauszus@gmail.com

from __future__ import absolute_import, print_function

import argparse
import os
import struct
import sys
import time
import logging
from typing import Dict, List, Tuple, Union
import can
from can import __version__
import cantools
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import numpy as np





logger = logging.getLogger('can.serial')

try:
    import curses
    from curses.ascii import ESC as KEY_ESC, SP as KEY_SPACE
except ImportError:
    # Probably on windows
    logger.warning("You won't be able to use the viewer program without "
                   "curses installed!")
    curses = None


class CanViewer:

    def __init__(self, stdscr, bus, data_structs, testing=False):
        self.stdscr = stdscr
        self.bus = bus
        self.data_structs = data_structs

        # Initialise the ID dictionary, start timestamp, scroll and variable for pausing the viewer
        self.ids = {}
        self.start_time = None
        self.scroll = 0
        self.paused = False

        # Get the window dimensions - used for resizing the window
        self.y, self.x = self.stdscr.getmaxyx()

        # Do not wait for key inputs, disable the cursor and choose the background color automatically
        self.stdscr.nodelay(True)
        curses.curs_set(0)
        curses.use_default_colors()

        # Used to color error frames red
        curses.init_pair(1, curses.COLOR_RED, -1)

        if not testing:  # pragma: no cover
            self.run()

    def run(self):
        curret = os.getcwd()
 
        db = cantools.database.load_file(curret+'/can/db/BoschIMU.dbc')
        db_leddar = cantools.database.load_file(curret+'/can/db/LeddarDB.dbc') 
        pub_imu = rospy.Publisher('imu_bosh', Imu ,queue_size=1)
        leddar_pub = rospy.Publisher("/leddartech", PointCloud2,queue_size=1)
        rospy.init_node('imu_bosh')
        try:
            msg_start = can.Message(arbitration_id=1856, data=[5, 1, 0, 0, 0, 0, 0, 0], extended_id=False)
            self.bus.send(msg_start)
        except can.CanError:
            print("Message NOT sent")
	imu_counter =0
	leddar_counter =[0,0,0,0,0,0,0,0]
        messaggio_imu = Imu()
        ax=0
        ay=0
        az=0
        distance00 =0
        distance01 =0
        distance02 =0
        distance03 =0
        distance04 =0
        distance05 =0
        distance06 =0
        distance07 =0
        distance08 =0
        distance09 =0
        distance010 =0
        distance011 =0
        distance012 =0
        distance013 =0
        distance014 =0
        distance015 =0
        heigh = 0.35
        step = 1.74533/16 #radianti
        start_angle = (3.14159-1.74533)/2
        
        
        roll_rate=0 
        yaw_rate =0

        while 1:
            # Do not read the CAN-Bus when in paused mode	
            if not self.paused:
                # Read the CAN-Bus and draw it in the terminal window
                msg = self.bus.recv(timeout=1. / 1000.)
                #self.bus.send(msg_start)
                if msg is not None:
                    try:
                        #print (msg.arbitration_id)
                        print ("\r")
                        if (msg.arbitration_id == 0x376 or msg.arbitration_id == 0x372 or msg.arbitration_id == 0x380 ):
                            decoded_message = (db.decode_message((msg.arbitration_id ), msg.data))
                        
                        #print (msg.arbitration_id)
                        #print (decoded_message)
                            if (msg.arbitration_id ==376): #ax
                                ax = decoded_message['Ax']
                                roll_rate = decoded_message['RollRate']
                                imu_counter+=1
                            if (msg.arbitration_id ==372): #ay
                                ay = decoded_message['Ay']
                                yaw_rate = decoded_message['YawRate']
                                imu_counter+=1
                            if (msg.arbitration_id ==380): #az
                                az = decoded_message['Az']
                                imu_counter+=1
                            
                            messaggio_imu.linear_acceleration.x = ax
                            messaggio_imu.linear_acceleration.y = ay
                            messaggio_imu.linear_acceleration.z = az
                            messaggio_imu.angular_velocity.x = roll_rate
                            messaggio_imu.angular_velocity.y = yaw_rate
                            if (imu_counter>=3):
                                pub_imu.publish(messaggio_imu)
                                imu_counter=0
                            #print ("ax: ", ax, " ay: ",ay, " az:",az, "\r")
                            print ("ax: %1.5f, ay: %1.5f, az: %1.5f\r" % (ax,ay,az) )
                            #print ("\r")
                            
                        if (msg.arbitration_id > 0x751 and msg.arbitration_id < 0x760  ):
                            decoded_message = (db_leddar.decode_message((msg.arbitration_id ), msg.data))
                            if (msg.arbitration_id ==0x752):
                                distance00 = float (decoded_message['Distance00'])/100
                                distance01 = float (decoded_message['Distance01'])/100
                                leddar_counter[0]=1
                                #print ("1")
                            if (msg.arbitration_id ==0x753):
                                distance02 = float(decoded_message['Distance02'])/100
                                distance03 = float(decoded_message['Distance03'])/100
                                leddar_counter[1]=1
                                #print ("2")
                            if (msg.arbitration_id ==0x754):
                                distance04 = float(decoded_message['Distance04'])/100
                                distance05 = float(decoded_message['Distance05'])/100
                                leddar_counter[2]=1
                                #print ("3")
                            if (msg.arbitration_id ==0x755):
                                distance06 = float(decoded_message['Distance06'])/100
                                distance07 = float(decoded_message['Distance07'])/100
                                leddar_counter[3]=1
                                #print ("4")
                            if (msg.arbitration_id ==0x756):
                                distance08 = float(decoded_message['Distance08'])/100
                                distance09 = float(decoded_message['Distance09'])/100
                                leddar_counter[4]=1
                                #print ("5")
                            if (msg.arbitration_id ==0x757):
                                distance10 = float(decoded_message['Distance10'])/100
                                distance11 = float(decoded_message['Distance11'])/100
                                leddar_counter[5]=1
                                #print ("6")
                            if (msg.arbitration_id ==0x758):
                                distance12 = float(decoded_message['Distance12'])/100
                                distance13 = float(decoded_message['Distance13'])/100
                                leddar_counter[6]=1
                                #print ("7")
                            if (msg.arbitration_id ==0x759):
                                distance14 = float(decoded_message['Distance14'])/100
                                distance15 = float(decoded_message['Distance15'])/100
                                leddar_counter[7]=1
                                #print ("8")
                                
                            if sum(leddar_counter)>=8:
                                leddar_counter =[0,0,0,0,0,0,0,0]
                                cloud_points = [[distance00*np.sin (step*0+start_angle) , distance00*np.cos (step*0+start_angle), heigh], \
                                [distance01*np.sin (step*1+start_angle) , distance01*np.cos (step*1+start_angle), heigh], \
                                [distance02*np.sin (step*2+start_angle) , distance02*np.cos (step*2+start_angle), heigh], \
                                [distance03*np.sin (step*3+start_angle) , distance03*np.cos (step*3+start_angle), heigh], \
                                [distance04*np.sin (step*4+start_angle) , distance04*np.cos (step*4+start_angle), heigh], \
                                [distance05*np.sin (step*5+start_angle) , distance05*np.cos (step*5+start_angle), heigh], \
                                [distance06*np.sin (step*6+start_angle) , distance06*np.cos (step*6+start_angle), heigh], \
                                [distance07*np.sin (step*7+start_angle) , distance07*np.cos (step*7+start_angle), heigh], \
                                [distance08*np.sin (step*8+start_angle) , distance08*np.cos (step*8+start_angle), heigh], \
                                [distance09*np.sin (step*9+start_angle) , distance09*np.cos (step*9+start_angle), heigh], \
                                [distance10*np.sin (step*10+start_angle) , distance10*np.cos (step*10+start_angle), heigh], \
                                [distance11*np.sin (step*11+start_angle) , distance11*np.cos (step*11+start_angle), heigh], \
                                [distance12*np.sin (step*12+start_angle) , distance12*np.cos (step*12+start_angle), heigh], \
                                [distance13*np.sin (step*13+start_angle) , distance13*np.cos (step*13+start_angle), heigh], \
                                [distance14*np.sin (step*14+start_angle) , distance14*np.cos (step*14+start_angle), heigh], \
                                [distance15*np.sin (step*15+start_angle) , distance15*np.cos (step*15+start_angle), heigh]]
                                header = std_msgs.msg.Header()
                                header.stamp = rospy.Time.now()
                                header.frame_id = 'leddartech'
                                scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
                                leddar_pub.publish(scaled_polygon_pcl)
                                print ("distance: %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \r" % (distance00, distance01,distance02,distance03, \
                                distance04, distance05,distance06,distance07, distance08,distance09,distance10, distance11,distance12,distance13, distance14,distance15) )
                                
                                
                                
                                
                                
                            #print (decoded_message)
                            #print ("\r")
                    except Exception as e: 
                        print(e)

                    
                    #self.draw_can_bus_message(msg)
            else:
                # Sleep 1 ms, so the application does not use 100 % of the CPU resources
                time.sleep(1. / 1000.)

           
        # Shutdown the CAN-Bus interface
        self.bus.shutdown()

    # Unpack the data and then convert it into SI-units
    @staticmethod
    def unpack_data(cmd, cmd_to_struct, data):  # type: (int, Dict, bytes) -> List[Union[float, int]]
        if not cmd_to_struct or len(data) == 0:
            # These messages do not contain a data package
            return []

        for key in cmd_to_struct.keys():
            if cmd == key if isinstance(key, int) else cmd in key:
                value = cmd_to_struct[key]
                if isinstance(value, tuple):
                    # The struct is given as the fist argument
                    struct_t = value[0]  # type: struct.Struct

                    # The conversion from raw values to SI-units are given in the rest of the tuple
                    values = [d // val if isinstance(val, int) else float(d) / val
                              for d, val in zip(struct_t.unpack(data), value[1:])]
                else:
                    # No conversion from SI-units is needed
                    struct_t = value  # type: struct.Struct
                    values = list(struct_t.unpack(data))

                return values
        else:
            raise ValueError('Unknown command: 0x{:02X}'.format(cmd))

    





def parse_args(args):
    # Python versions >= 3.5
    kwargs = {}
    if sys.version_info[0] * 10 + sys.version_info[1] >= 35:  # pragma: no cover
        kwargs = {'allow_abbrev': False}

    # Parse command line arguments
    parser = argparse.ArgumentParser('python -m can.viewer',
                                     description='A simple CAN viewer terminal application written in Python',
                                     epilog='R|Shortcuts: '
                                            '\n        +---------+-------------------------+'
                                            '\n        |   Key   |       Description       |'
                                            '\n        +---------+-------------------------+'
                                            '\n        | ESQ/q   | Exit the viewer         |'
                                            '\n        | c       | Clear the stored frames |'
                                            '\n        | s       | Sort the stored frames  |'
                                            '\n        | SPACE   | Pause the viewer        |'
                                            '\n        | UP/DOWN | Scroll the viewer       |'
                                            '\n        +---------+-------------------------+',
                                      add_help=False, **kwargs)

    optional = parser.add_argument_group('Optional arguments')

    optional.add_argument('-h', '--help', action='help', help='Show this help message and exit')

    optional.add_argument('--version', action='version', help="Show program's version number and exit",
                          version='%(prog)s (version {version})'.format(version=__version__))

    # Copied from: https://github.com/hardbyte/python-can/blob/develop/can/logger.py
    optional.add_argument('-b', '--bitrate', type=int, help='''Bitrate to use for the given CAN interface''')

    optional.add_argument('-c', '--channel', help='''Most backend interfaces require some sort of channel.
                          For example with the serial interface the channel might be a rfcomm device: "/dev/rfcomm0"
                          with the socketcan interfaces valid channel examples include: "can0", "vcan0".
                          (default: use default for the specified interface)''')

    optional.add_argument('-d', '--decode', dest='decode',
                          help='R|Specify how to convert the raw bytes into real values.'
                               '\nThe ID of the frame is given as the first argument and the format as the second.'
                               '\nThe Python struct package is used to unpack the received data'
                               '\nwhere the format characters have the following meaning:'
                               '\n      < = little-endian, > = big-endian'
                               '\n      x = pad byte'
                               '\n      c = char'
                               '\n      ? = bool'
                               '\n      b = int8_t, B = uint8_t'
                               '\n      h = int16, H = uint16'
                               '\n      l = int32_t, L = uint32_t'
                               '\n      q = int64_t, Q = uint64_t'
                               '\n      f = float (32-bits), d = double (64-bits)'
                               '\nFx to convert six bytes with ID 0x100 into uint8_t, uint16 and uint32_t:'
                               '\n  $ python -m can.viewer -d "100:<BHL"'
                               '\nNote that the IDs are always interpreted as hex values.'
                               '\nAn optional conversion from integers to real units can be given'
                               '\nas additional arguments. In order to convert from raw integer'
                               '\nvalues the values are divided with the corresponding scaling value,'
                               '\nsimilarly the values are multiplied by the scaling value in order'
                               '\nto convert from real units to raw integer values.'
                               '\nFx lets say the uint8_t needs no conversion, but the uint16 and the uint32_t'
                               '\nneeds to be divided by 10 and 100 respectively:'
                               '\n  $ python -m can.viewer -d "101:<BHL:1:10.0:100.0"'
                               '\nBe aware that integer division is performed if the scaling value is an integer.'
                               '\nMultiple arguments are separated by spaces:'
                               '\n  $ python -m can.viewer -d "100:<BHL" "101:<BHL:1:10.0:100.0"'
                               '\nAlternatively a file containing the conversion strings separated by new lines'
                               '\ncan be given as input:'
                                '\n  $ cat file.txt'
                                '\n      100:<BHL'
                                '\n      101:<BHL:1:10.0:100.0'
                                '\n  $ python -m can.viewer -d file.txt',
                          metavar='{<id>:<format>,<id>:<format>:<scaling1>:...:<scalingN>,file.txt}',
                          nargs=argparse.ONE_OR_MORE, default='')

    optional.add_argument('-f', '--filter', help='R|Space separated CAN filters for the given CAN interface:'
                          '\n      <can_id>:<can_mask> (matches when <received_can_id> & mask == can_id & mask)'
                          '\n      <can_id>~<can_mask> (matches when <received_can_id> & mask != can_id & mask)'
                          '\nFx to show only frames with ID 0x100 to 0x103 and 0x200 to 0x20F:'
                          '\n      python -m can.viewer -f 100:7FC 200:7F0'
                          '\nNote that the ID and mask are alway interpreted as hex values',
                          metavar='{<can_id>:<can_mask>,<can_id>~<can_mask>}', nargs=argparse.ONE_OR_MORE, default='')

    optional.add_argument('-i', '--interface', dest='interface',
                          help='R|Specify the backend CAN interface to use.',
                          choices=sorted(can.VALID_INTERFACES))

    # Print help message when no arguments are given
    if len(args) == 0:
        parser.print_help(sys.stderr)
        import errno
        raise SystemExit(errno.EINVAL)

    parsed_args = parser.parse_args(args)

    can_filters = []
    if len(parsed_args.filter) > 0:
        # print('Adding filter/s', parsed_args.filter)
        for flt in parsed_args.filter:
            # print(filter)
            if ':' in flt:
                _ = flt.split(':')
                can_id, can_mask = int(_[0], base=16), int(_[1], base=16)
            elif '~' in flt:
                can_id, can_mask = flt.split('~')
                can_id = int(can_id, base=16) | 0x20000000  # CAN_INV_FILTER
                can_mask = int(can_mask, base=16) & 0x20000000  # socket.CAN_ERR_FLAG
            else:
                raise argparse.ArgumentError(None, 'Invalid filter argument')
            can_filters.append({'can_id': can_id, 'can_mask': can_mask})

    # Dictionary used to convert between Python values and C structs represented as Python strings.
    # If the value is 'None' then the message does not contain any data package.
    #
    # The struct package is used to unpack the received data.
    # Note the data is assumed to be in little-endian byte order.
    # < = little-endian, > = big-endian
    # x = pad byte
    # c = char
    # ? = bool
    # b = int8_t, B = uint8_t
    # h = int16, H = uint16
    # l = int32_t, L = uint32_t
    # q = int64_t, Q = uint64_t
    # f = float (32-bits), d = double (64-bits)
    #
    # An optional conversion from real units to integers can be given as additional arguments.
    # In order to convert from raw integer value the real units are multiplied with the values and similarly the values
    # are divided by the value in order to convert from real units to raw integer values.
    data_structs = {}  # type: Dict[Union[int, Tuple[int, ...]], Union[struct.Struct, Tuple, None]]
    if len(parsed_args.decode) > 0:
        if os.path.isfile(parsed_args.decode[0]):
            with open(parsed_args.decode[0], 'r') as f:
                structs = f.readlines()
        else:
            structs = parsed_args.decode

        for s in structs:
            tmp = s.rstrip('\n').split(':')

            # The ID is given as a hex value, the format needs no conversion
            key, fmt = int(tmp[0], base=16), tmp[1]

            # The scaling
            scaling = []  # type: list
            for t in tmp[2:]:
                # First try to convert to int, if that fails, then convert to a float
                try:
                    scaling.append(int(t))
                except ValueError:
                    scaling.append(float(t))

            if scaling:
                data_structs[key] = (struct.Struct(fmt),) + tuple(scaling)
            else:
                data_structs[key] = struct.Struct(fmt)
            # print(data_structs[key])

    return parsed_args, can_filters, data_structs


def main():  # pragma: no cover
    parsed_args, can_filters, data_structs = parse_args(sys.argv[1:])

    config = {'single_handle': True}
    if can_filters:
        config['can_filters'] = can_filters
    if parsed_args.interface:
        config['interface'] = parsed_args.interface
    if parsed_args.bitrate:
        config['bitrate'] = parsed_args.bitrate

    # Create a CAN-Bus interface
    bus = can.Bus(parsed_args.channel, **config)
    # print('Connected to {}: {}'.format(bus.__class__.__name__, bus.channel_info))

    curses.wrapper(CanViewer, bus, data_structs)


if __name__ == '__main__':  # pragma: no cover
    # Catch ctrl+c
    try:
        main()
    except KeyboardInterrupt:
        pass
