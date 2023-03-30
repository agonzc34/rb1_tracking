from setuptools import setup
import glob

package_name = 'ros_yolov8'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/net_props', glob.glob('net_props/*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*'))
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='athykxz',
    maintainer_email='athykxz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_yolov8 = ros_yolov8.ros_yolov8:main',
            'webcam = ros_yolov8.webcam:main'
        ],
    },
)
