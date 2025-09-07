from setuptools import setup

package_name = 'gbr_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.packages', package_name + '.direct_control'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='GBR control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynsys_program = gbr_control.dynsys_program:main',
            'dynsys_program2 = gbr_control.dynsys_program2:main',
            'motor_pwm_node = gbr_control.motor_pwm_node:main',
            'motor_pwm_subscriber = gbr_control.motor_pwm_subscriber:main',
            'gbr_direct_interface = gbr_control.gbr_direct_interface_python_example:main',
            'gbr_keyboard_demo = gbr_control.gbr_keyboard_demo:main'
        ],
    },
)

