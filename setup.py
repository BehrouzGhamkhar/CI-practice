from setuptools import find_packages, setup

package_name = 'ros_SM'

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
    maintainer='behrouz',
    maintainer_email='behrouz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"robot_sm = ros_SM.robot_sm:main",
        	"battery = ros_SM.battery:main",
         	"collision = ros_SM.collision:main"
        ],
    },
)
