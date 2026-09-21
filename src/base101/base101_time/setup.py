from setuptools import find_packages, setup

package_name = 'base101_time'

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
    maintainer='author',
    maintainer_email='todo@todo.com',
    description="Host-side responder for the Axon 2 firmware's time-sync probes.",
    license='TODO',
    entry_points={
        'console_scripts': [
            'time_sync = base101_time.time_sync:main',
        ],
    },
)
