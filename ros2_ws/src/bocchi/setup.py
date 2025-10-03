from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bocchi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    package_data={
        'bocchi': [
            'templates/*',
            'static/css/*',
            'static/js/*'
        ],
    },
    include_package_data=True,
    install_requires=[
        'setuptools',
        'Flask==2.0.3',
        'Flask-CORS==3.0.10',
        'websockets==11.0.3',
        'requests==2.25.1',
        'sseclient-py==1.8.0',
        'Mako==1.1.6'
    ],
    zip_safe=True,
    maintainer='kitakitadesu',
    maintainer_email='bookshorse@outlook.jp',
    description='keyboard publisher',
    license='MIT',
    tests_require=['pytest', 'websockets', 'pytest-asyncio', 'pytest-cov'],
    test_suite='pytest',
    entry_points={
        'console_scripts': [
            'publisher = bocchi.publisher:main',
            'subscriber = bocchi.subscriber:main'
        ],
    },
)