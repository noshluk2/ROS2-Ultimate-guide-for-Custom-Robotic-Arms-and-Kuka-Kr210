#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import os


urdf_file=os.path.join("urdf","kr210.urdf")

kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_file)
T=kuka_robot.forward_kinematics([0] * 9)

print("\nTransformation Matrix :\n",T)
angles=kuka_robot.inverse_kinematics([2.2,1.2,1.2])
print("\nAngles Computer\n",angles)
angles=np.delete(angles, [0,7,8])
print("\nCorrected Angles \n",list(angles))