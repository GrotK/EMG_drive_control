from setuptools import setup

package_name = 'EMG_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='EMG gesture control for TurtleBot in ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offline_publisher = scripts.offline_publisher:main',
            'main = scripts.main:main',
            'gesture_to_cmd = scripts.gesture_to_cmd:main',
        ],
    },
)
