#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from carmaker_node.msg import sub_udp


msgs = ''' 1 : maneuver
        s : stop
        w : acc +
        x : acc -
        a : left turn
        d : right turn
        j : warning light off
        k : warning light on
        i : light off
        o : left light on
        p : right light on
        
        
        
         '''





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
    print(msgs)
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


            if key == 'j':
                udp.Lights_Hazard = 0
                print('warning light off')  



            if key == 'k':
                udp.Lights_Hazard = 1
                print('warning light on')  

            if key == 'i':
                udp.Lights_Indicator = 0
                print('light off') 

            if key == 'o':
                udp.Lights_Indicator = 1
                print('left light') 


            if key == 'p':
                udp.Lights_Indicator = -1
                print('right light') 

            else:
                if (key == '\x03'):
                    #pub_start.publish(0)
                    break
            mode_pub.publish(udp)


    except rospy.ROSInterruptException:
        pass
