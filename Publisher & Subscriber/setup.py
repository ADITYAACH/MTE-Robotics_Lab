from setuptools import setup
from glob import glob
import os

package_name = 'my_package'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achanti',
    maintainer_email='achanti@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sample = my_package.sample:main',
        'publisher = my_package.publisher:main',  
        'subscriber = my_package.subscriber:main'       
        ],
    },
 )