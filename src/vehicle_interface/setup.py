from setuptools import find_packages, setup

package_name = 'vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carola Adrados Herrero',
    maintainer_email='adradoshc@gmail.com',
    description='Vehicle interface: normalised commands → PWM µs, deadman, ESC arming',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # executable name matches what launch files use
            'vehicle_interface = vehicle_interface.interface_node:main',
        ],
    },
)
