from setuptools import setup

package_name = 'sensors'

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
    maintainer_email='kraft2k@yandex.ru',
    description='Publishes temperature from sensor DHT11 and  distance from Benewake DE-LIDAR TF02_PRO (in cm, up to 40 meters)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "temperature_publisher = sensors.temperature_publisher:main",
            "temperature_subscriber = sensors.temperature_subscriber:main",
            "distance_publisher = sensors.distance_publisher:main",
            "distance_subscriber = sensors.distance_subscriber:main"
        ],
    },
)
