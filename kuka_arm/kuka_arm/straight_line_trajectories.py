import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

import ikpy.chain
import sys 
import numpy as np
from time import time, sleep

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','left_gripper_finger_joint','right_gripper_finger_joint']
        
        ## Toolbox interface
        self.robot_initialize()
        argv = sys.argv[1:] 
        self.inverse_kinematics_solution(float(argv[0])+0.1,float(argv[1]),float(argv[2]))
            

    def timer_callback(self):
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        ## adding newly created point into trajectory message
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(bazu_trajectory_msg)
        print("\nTrajectory Sent !\n")

    def robot_initialize(self):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file("/home/luqman/r2_ra_ws/src/kuka_arm/urdf/kuka_model.urdf")
    
    def inverse_kinematics_solution(self,x,y,z):
        angles=self.kuka_robot.inverse_kinematics([x,y,z])
        angles=np.delete(angles, [0,7,8])
        print("\nClaw Closed\n")
        self.goal_positions = list(np.append(angles ,[0.00, 0.0]) ) 
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)



def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()