from setuptools import setup

package_name = 'hardware_control'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_rotation = hardware_control.camera_rotation:main",
            "hold_distance = hardware_control.hold_distance:main",
            "warning_lights_control = hardware_control.warning_lights_control:main"
        ],
    },
)
