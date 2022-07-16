from setuptools import setup

package_name = 'demo_trainee22_1'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leonardo dos Santos',
    maintainer_email='leohmcs@gmail.com',
    description='Demo for the navigation class during trainee of AVANT UFMG',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = demo_trainee22_1.odom:main',
            'plot_position = demo_trainee22_1.plot_position:main', 
        ],
    },
)
