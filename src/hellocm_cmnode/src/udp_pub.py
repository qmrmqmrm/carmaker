#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from hellocm_msgs.msg import sub_udp
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('test_pub')
    mode_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
    udp = sub_udp()
    udp.Ax = 0
    udp.SteeringWheel = 0
    try:
        while (1):
            key = getKey()
            if key == '1':
                udp.VC_SwitchOn = 0
                # mode_pub.publish(udp)
                print('maneuver')   

            if key == 's':
                udp.VC_SwitchOn = 1
                udp.GearNo = -9
                udp.Ax = 0
                udp.SteeringWheel = 0
                # mode_pub.publish(udp)
                print('stop')
            if key == 'w':
                udp.VC_SwitchOn = 1
                if udp.Ax >= 0:
                	udp.GearNo = 1
                else:
                	udp.GearNo = -1
                udp.Ax += 1

                print('Ax +') 

            if key == 'x':
                udp.VC_SwitchOn = 1
                if udp.Ax >= 0:
                	udp.GearNo = 1
                else:
                	udp.GearNo = -1
                udp.Ax -= 1
                # mode_pub.publish(udp)
                print('Ax -')                

            if key == 'a':
                udp.VC_SwitchOn = 1
                udp.SteeringWheel += 0.1
                # mode_pub.publish(udp)
                print('SteeringWheel +')   

            if key == 'd':
                udp.VC_SwitchOn = 1
                udp.SteeringWheel -= 0.1
                # mode_pub.publish(udp)
                print('SteeringWheel -')   

            else:
                if (key == '\x03'):
                    #pub_start.publish(0)
                    break
            mode_pub.publish(udp)


    except rospy.ROSInterruptException:
        pass
