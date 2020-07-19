from setuptools import setup

package_name = 'tsim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/tsim_launch.py']),
        ('share/' + package_name, ['config/param.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jungpil YU',
    maintainer_email='jungpil.yu@gmail.com',
    description='The Simplest Turtlesim',
    license='Apache license 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = tsim.commander:main',
            'motors = tsim.motors:main',
            'locator = tsim.locator:main',
            'requester = tsim.requester:main',
            'autopilot = tsim.autopilot:main',
        ],
    },
)
