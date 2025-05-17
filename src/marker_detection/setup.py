from setuptools import find_packages, setup

package_name = 'marker_detection'

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
    maintainer='plasminforge',
    maintainer_email='plasminforge@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brightness_threshold = marker_detection.brightness_threshold:main',
            '3dify = marker_detection.3dify:main'
        ],
    },
)
