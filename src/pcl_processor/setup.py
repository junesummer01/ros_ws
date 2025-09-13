from setuptools import find_packages, setup

package_name = 'pcl_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='ojh',
    maintainer_email='ogane98@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_visualizer = pcl_processor.odometry_visualizer:main',
        ],
    },
)
