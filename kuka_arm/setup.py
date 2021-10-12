from setuptools import setup
from glob import glob
import os
package_name = 'kuka_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),

        (os.path.join('share', package_name,'launch'), glob('launch/*')),

        (os.path.join('share', package_name,'config'), glob('config/*')),

        (os.path.join('share', package_name,'meshes/visual/'), glob('meshes/visual/*')),

        (os.path.join('share', package_name,'meshes/collision/'), glob('meshes/collision/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'objGrip_move = kuka_arm.object_throw:main',
            'obj_pos = kuka_arm.object_position:main',
            'obj_grip = kuka_arm.object_grab:main',
            'origin_traj = kuka_arm.trajectory_commander_a:main',
            'random_traj = kuka_arm.trajectory_commander_b:main',
            'ik_traj_xyz_arg = kuka_arm.inverse_kinematics:main'
        ],
    },
)
