# ~/limo_ws/src/guide_dog_robot/setup.py

from setuptools import setup
import os
from glob import glob

package_name = 'guide_dog_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.behaviors'], # behaviors 패키지 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 폴더 설치 설정 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Guide Dog Robot Project using Python BT',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 터미널에서 실행할 명령어 등록
            'main_bt_node = guide_dog_robot.main_bt_node:main',
        ],
    },
)
