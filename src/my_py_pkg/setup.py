from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='us-robot',
    maintainer_email='us-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "imu_node = my_py_pkg.imu_sensor:main",
            "controller_node = my_py_pkg.robot_controller:main",
            "wheels_node = my_py_pkg.wheels_controller:main"
        ],
    },
)
