from setuptools import setup

package_name = 'log2json'

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
    maintainer='robotica',
    maintainer_email='fjrodl@unileon.es',
    description='A ROS 2 package for creating json messages',
    license='Apache',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log2json_node = log2json.log2json_node:main'
        ],
    },
)
