#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
speed up/down - u/j
left/right - i/o
stop : k/l
CTRL-C to quit
"""

speedBindings = {
        'u':100,
		'j':-100,
        'U':100,
        'J':-100,
	       }
turnBindings = {
        'i':3,
		'o':-3,
        'I':3,
        'O':-3,
	       }
stopBindings = {
        'k':0,
		'l':-0,
        'K':0,
        'l':-0,
	       }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed_val = 0
turn_val = 81

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    speed = rospy.Publisher('/manual_control/speed',Int16, queue_size = 1)
    turn  = rospy.Publisher('/manual_control/steering',Int16, queue_size = 1)
    rospy.init_node('teleop_keyboard_model_car')

    try:
        print msg
        while(1):
            global speed_val
            global turn_val
            key = getKey()
            if key in speedBindings.keys():
                #print "mv key!!",key,"!!"
                speed_val +=  speedBindings[key]
                if (speed_val > 800):
                    speed_val = 800
                elif (speed_val <0):
                    speed_val = 0
            elif key in turnBindings.keys():
                #print "sp key!!",key,"!!"
                turn_val += turnBindings[key]
                if turn_val>160:
                    turn_val = 160;
                elif turn_val <0:
                    turn_val =0;
            elif key in stopBindings.keys():
                turn_val = 0
                speed_val = 0
            if (key == '\x03'):
                speed.publish(0)
                turn.publish(0)
                break

            print vels(speed_val,turn_val)
            speed.publish(-speed_val)
            turn.publish(turn_val)

    except:
        print "something is wrong"

    finally:
        #publish zero for everything
        speed.publish(0)
        turn.publish(0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
