from setuptools import find_packages, setup

package_name = 'otherNodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/config', ['config/joystick.yaml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/config', ['config/mapping.yaml']),
        ('share/' + package_name + '/config', ['config/navigation.yaml']),
        ('share/' + package_name + '/models', ['models/robots/pioneer.urdf'])
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='benlilburne@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_switch = otherNodes.dead_switch:main'
        ],
    },
)
