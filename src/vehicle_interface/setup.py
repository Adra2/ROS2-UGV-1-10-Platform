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
    maintainer='yelos',
    maintainer_email='yelos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_interface = vehicle_interface.interface_node:main',
            'servo_node = vehicle_interface.servo_node:main',
            'esc_node   = vehicle_interface.esc_node:main',
        ],
    },
)
