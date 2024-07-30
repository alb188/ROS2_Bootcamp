from setuptools import find_packages, setup

package_name = 'pubsub_76'

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
    maintainer='Alex',
    maintainer_email='albela275@gmail.com',
    description='7 dimensional vector publisher and 6 dimensional vector subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pubsub_76.pub_76_function:main',
            'listener = pubsub_76.sub_76_function:main',
        ],
    },
)
