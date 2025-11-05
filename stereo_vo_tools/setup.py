from setuptools import setup, find_packages

package_name = 'stereo_vo_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='dev@example.com',
    description='Python tools for stereo VO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_stereo = stereo_vo_tools.calibrate_stereo:main',
            'eval_kitti_like = stereo_vo_tools.eval_kitti_like:main',
        ],
    },
)
