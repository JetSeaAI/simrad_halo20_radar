import os
from setuptools import setup, find_packages
from Cython.Build import cythonize
from glob import glob

package_name = 'halo_radar_visualize'
files = [package_name + "/halo_radar_visualize.py", package_name + "/radar_interface.py",
          package_name + "/halo_radar_data_cropper.py", package_name + "/halo_radar_merge_scan.py",]

setup(
    ext_modules=cythonize(files, compiler_directives={'language_level': "3"}, force=True, quiet=True),
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz2'),
          glob(os.path.join('rviz2', '*.rviz'))),
    ],
    install_requires=['setuptools', "wheel", "Cython"],
    zip_safe=True,
    maintainer='chrisyang',
    maintainer_email='chris.yang@jetseaai.com',
    description='ROS 2 package for Halo Radar',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'halo_radar_visualize = halo_radar_visualize.halo_radar_visualize:main',
            'halo_radar_control_panel = halo_radar_visualize.halo_radar_control_panel:main',
            'halo_radar_data_cropper = halo_radar_visualize.halo_radar_data_cropper:main',
            'halo_radar_merge_scan = halo_radar_visualize.halo_radar_merge_scan:main',
        ],
    },
)