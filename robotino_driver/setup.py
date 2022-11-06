from setuptools import setup

package_name = 'robotino_driver'

data_files = []
data_files.append(('share.ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robotino_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_site.wbt']))
data_files.append(('share/' + package_name + '/protos', ['protos/CameraHandle.proto']))
data_files.append(('share/' + package_name + '/resource', ['resource/Aruco marker DICT_4X4_50 ID 0.png']))
data_files.append(('share/' + package_name + '/resource', ['resource/robotino_3.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrOleg5',
    maintainer_email='oleg.brylyov@mail.ru',
    description='Webots Robotino Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotino_driver = robotino_driver.robotino_driver:main'
        ],
    },
)
