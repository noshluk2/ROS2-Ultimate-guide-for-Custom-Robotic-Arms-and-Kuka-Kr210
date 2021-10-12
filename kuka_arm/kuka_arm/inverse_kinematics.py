import numpy
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

from spatialmath.base import *
from spatialmath import SE3
import math
import roboticstoolbox as rtb
import sys 
import numpy as np


# Near to the ground to check grab
#-1.5 1 -1.5 c

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','left_gripper_finger_joint','right_gripper_finger_joint']
        
        ## Toolbox interface
        self.robot_dh_table()
        argv = sys.argv[1:] 
        self.inverse_kinematics_solution(float(argv[0]),float(argv[1]),float(argv[2]),argv[3] ) # point 
    
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
        # print("\nTrajectory Sent !\n")

    def robot_dh_table(self):
        Link_1=rtb.DHLink(0.75 , -math.pi/2,   0  , 0.35)
        Link_2=rtb.DHLink(0    ,    0      ,   0  , 1.25)
        Link_3=rtb.DHLink(0    , -math.pi/2,   0  , -0.054)
        Link_4=rtb.DHLink(1.5  ,  math.pi/2,   0  , 0)
        Link_5=rtb.DHLink(0    , -math.pi/2,   0  , 0)
        Link_6=rtb.DHLink(0.303,    0      ,   0  , 0)
        self.kuka_robot= rtb.DHRobot([Link_1 ,Link_2,Link_3,Link_4,Link_5,Link_6])
    
    def get_fk_solution(self,angles_rad):
        angles=np.array(angles_rad)
        angles_rad=np.radians(angles)
        T=self.kuka_robot.fkine(angles_rad)
        print("\nTransformation Matrix :\n",T)
    
    def inverse_kinematics_solution(self,x,y,z,claw):
        point = SE3(x,y,z)
        if (claw=="o"):
            print("\nClaw Open\n")
            self.goal_positions = list(np.append(self.kuka_robot.ikine_LM(point)[0] ,[-0.01,-0.01]) )
        else:
            print("\nClaw Closed\n")
            self.goal_positions = list(np.append(self.kuka_robot.ikine_LM(point)[0] ,[0.06, 0.06]) )
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)



def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()