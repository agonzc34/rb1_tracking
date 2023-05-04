from setuptools import setup

package_name = 'rb1_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athykxz',
    maintainer_email='athykxz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rb1_tracker = rb1_tracker.rb1_tracker:main',
            'ray_pub = rb1_tracker.ray_pub:main',
            'rb1_tracker_noyolo = rb1_tracker.rb1_tracker_noyolo:main',
            'image = rb1_tracker.image:main' 
        ],
    },
)
