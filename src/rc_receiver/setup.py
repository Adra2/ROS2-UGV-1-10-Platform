from setuptools import find_packages, setup

package_name = 'rc_receiver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Carola A.H.',
    maintainer_email='adradoshc@gmail.com',
    description='SBUS RC receiver node for ROS2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rc_receiver_node = rc_receiver.rc_receiver_node:main',
        ],
    },
)
