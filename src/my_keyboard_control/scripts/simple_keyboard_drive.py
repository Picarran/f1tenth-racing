#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import sys, tty, termios

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

if __name__ == "__main__":
    rospy.init_node("simple_keyboard_drive")
    pub = rospy.Publisher(
        "/vesc/low_level/ackermann_cmd_mux/output",
        AckermannDriveStamped,
        queue_size=1
    )

    speed = 0.0
    steer = 0.0



    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w':
            speed += 0.2
        elif key == 's':
            speed -= 0.2
        elif key == 'a':
            steer += 0.1
        elif key == 'd':
            steer -= 0.1
        elif key == 'x':
            speed = 0.0
            steer = 0.0
        elif key == 'q':
            break

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        pub.publish(msg)


