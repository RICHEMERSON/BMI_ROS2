from setuptools import setup

package_name = 'state_reader'

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
    maintainer='root',
    maintainer_email='cyli@ion.ac.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'psychopy_1D_center_out = state_reader.psychopy_presenter_1D_center_out:main',
        'psychopy_2D_center_out = state_reader.psychopy_presenter_2D_center_out:main',
        'psychopy_2D_center_out_continuous = state_reader.psychopy_presenter_2D_interception_continuous:main',
        'psychopy_2D_interception = state_reader.psychopy_presenter_2D_interception:main',
        'psychopy_2D_semi_interception = state_reader.psychopy_presenter_2D_interception_semi_old:main',
        'monkeylogic_server = state_reader.monkeylogic_interception:main',
        ],
    },
)
