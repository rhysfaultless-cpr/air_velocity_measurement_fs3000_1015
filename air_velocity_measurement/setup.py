from setuptools import find_packages, setup

package_name = 'air_velocity_measurement'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rhys Faultless',
    maintainer_email='rfaultless@clearpathrobotics.com',
    description='Measure air velocity in cm/s with a Sparkfun FS3000-1015 sensor',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'air_velocity_measurement = air_velocity_measurement.air_velocity_measurement:main',
        ],
    },
)
