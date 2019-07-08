from setuptools import setup

package_name = 'ros_mstar'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/m_star_planner_launch_example.py']),
        ('share/' + package_name, ['launch/simple_service_client_launch_example.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Chris Bollinger',
    author_email='bollingc@oregonstate.edu',
    maintainer='Chris Bollinger',
    maintainer_email='bollingc@oregonstate.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Multi-robot decentralized path planning code for OSU Mobile Robotics class project Spring 2019',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm_star_planner ='
            ' ros_mstar.m_star_planner:main',
            'explorer ='
            ' ros_mstar.explore_controller:main',
            'sink ='
            ' ros_mstar.topic_sink:main',
        ],
    },
)
