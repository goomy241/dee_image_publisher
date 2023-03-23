from setuptools import setup

package_name = 'dee_image_publisher'

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
    maintainer='dee',
    maintainer_email='dee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = dee_image_publisher.camera_publisher:main',
            'kitti_publisher = dee_image_publisher.kitti_publisher:main',
            'rosbag_publisher = dee_image_publisher.rosbag_publisher:main',
            'rosbag_play_test = dee_image_publisher.rosbagPlayTest:main',
        ],
    },
)
