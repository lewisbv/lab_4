from setuptools import setup

package_name = 'the_zucc'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/7785_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lewis Busch-Vogel',
    maintainer_email='lbuschvogel3@gatech.edu',
    description='My ROS2 Python nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go=the_zucc.processWaypoints:main',
            'drive=the_zucc.goToGoal:main',
            'avoid=the_zucc.getObjectRange:main',
        ],
    },
)
