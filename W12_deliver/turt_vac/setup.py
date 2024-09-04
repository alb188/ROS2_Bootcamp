from setuptools import find_packages, setup

package_name = 'turt_vac'

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
    description='Turtlesim vacuum cleaning script for circle',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bound = turt_vac.turt_vac_bound:main',
            'clean = turt_vac.turt_vac_spiral:main',
        ],
    },
)
