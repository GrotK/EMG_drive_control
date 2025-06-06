from setuptools import find_packages, setup

package_name = 'EMG_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture_pipeline.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'torch',        
        'numpy',
        'onnxruntime',               
        'rclpy',        
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'emg_listener = script.main:main',
           'emg_publisher = script.offline_publisher:main',
           'emg_turtle_controller = script.gesture_to_cmd:main'
        ],
    },
)
