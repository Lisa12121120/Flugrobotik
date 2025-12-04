from setuptools import find_packages, setup

package_name = 'crtp'
data_files = []
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))




setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinzenz Malke',
    maintainer_email='vinzenz@malke.info',
    description='Underlying logic for the crtp protocol used by the crazyflie',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
