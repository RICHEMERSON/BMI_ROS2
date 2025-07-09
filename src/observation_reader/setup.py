from setuptools import setup
import os

package_name = 'observation_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
        #(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='cyli@ion.ac.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'blackrock_ir_talker = observation_reader.blackrock_passive_ir_publisher:main',
         'blackrock_ir_talker_count = observation_reader.blackrock_passive_ir_publisher_count:main',
         'spikegadgt_ir_talker = observation_reader.SpikeGadgt_passive_ir_publisher:main',
        ],
    },
)
