from setuptools import find_packages, setup

package_name = 'pytest'

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
    maintainer='bharatjag',
    maintainer_email='jagwanib@my.erau.edu',
    description='ROS2 Simple Publisher and Subscriber',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_pub = pytest.simple_pub:main',
            'simple_subs= pytest.simple_subs:main',
            'simple_parameter= pytest.simple_parameter:main',
        ],
    },
)
