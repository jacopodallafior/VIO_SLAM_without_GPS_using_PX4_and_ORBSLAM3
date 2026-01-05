from setuptools import find_packages, setup

package_name = 'px4_orbslam_bridge'

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
    maintainer='jacopo',
    maintainer_email='jacopodallafior@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'droneflying = px4_orbslam_bridge.droneflying:main',
            'gz_orbslam_driver = px4_orbslam_bridge.gz_orbslam_driver:main',
            'vio_to_px4_ev_odom = px4_orbslam_bridge.vio_to_px4_ev_odom:main'

        ],
    },
)
