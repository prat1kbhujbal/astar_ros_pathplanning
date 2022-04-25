#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
import argparse
from astar import *
import time


class TurtleBot:

    def __init__(self):

        self.wh_radius = 0.038
        self.wheel_distance = 0.354

    def command(self, path):
        velocity_publisher = rospy.Publisher('/cmd_vel',
                                             Twist, queue_size=10)
        data = Twist()
        data.linear.x = 0.0
        data.angular.z = 0.0
        velocity_publisher.publish(data)
        step = 0
        rate = rospy.Rate(10)
        for action in path:
            while not rospy.is_shutdown():
                if step == 16:
                    data.linear.x = 0
                    data.angular.z = 0
                    velocity_publisher.publish(data)
                    break
                else:
                    vel_l = action[0]
                    vel_r = action[1]
                    ang_l = vel_l * 2 * math.pi / 60
                    ang_r = vel_r * 2 * math.pi / 60
                    th = (self.wh_radius / self.wheel_distance) * (ang_l - ang_r)
                    vel = (self.wh_radius / 2) * (ang_l + ang_r)
                    data.linear.x = vel * 7.9
                    data.angular.z = th * 10
                    velocity_publisher.publish(data)
                    step += 1
                    rate.sleep()
            step = 0


def main():
    """Main Function"""
    # unique node (using anonymous=True).
    rospy.init_node('controller', anonymous=True)
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--start', default=[1, 1, 0])
    Parser.add_argument('--goal', default=[2, 4, 0])
    Parser.add_argument('--rpm', default=[10, 10])
    Parser.add_argument('--clearance', default=0.1)

    args = Parser.parse_args(rospy.myargv()[1:])
    start_node = [float(i) for i in args.start[1:-1].split(',')]
    goal_node = [float(i) for i in args.goal[1:-1].split(',')]
    robot_rpms = [float(i) for i in args.rpm[1:-1].split(',')]
    clearance = float(args.clearance)

    # Map Grid
    map_grid = [10, 10]
    # Perfrom Astar
    time.sleep(5)
    moves = astar(
        start_node,
        goal_node,
        map_grid,
        robot_rpms,
        clearance)
    x = TurtleBot()
    x.command(moves)
    plt.show()


if __name__ == "__main__":
    main()
