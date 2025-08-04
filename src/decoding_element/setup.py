from setuptools import setup

package_name = 'decoding_element'

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
        'integrator = decoding_element.data_integrator:main',
        'trainer = decoding_element.decoding_element_model_trainer:main',
        'trainer_buffer = decoding_element.decoding_element_trainer_buffer:main',
        'predictor = decoding_element.decoding_element_predictor:main',
        'integrator_message_filters = decoding_element.data_integrator_message_filters:main'
        ],
    },
)
