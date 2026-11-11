from setuptools import setup
from glob import glob
setup(name='base101_mujoco',version='0.1.0',packages=['base101_mujoco'],
 data_files=[('share/ament_index/resource_index/packages',['resource/base101_mujoco']),
 ('share/base101_mujoco',['package.xml']),('share/base101_mujoco/launch',glob('launch/*.py')),
 ('share/base101_mujoco/config',glob('config/*'))],
 entry_points={'console_scripts':['sim = base101_mujoco.node:main']})
