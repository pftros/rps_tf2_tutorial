from setuptools import find_packages, setup

package_name = 'landmark_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damjan',
    maintainer_email='damjan@romb-technologies.hr',
    description='Illustrate the role of coordinate transforms in mobile robot localization.',
    license='Apache-2.0',
    extras_require={},
    entry_points={
        'console_scripts': [
            'localization = landmark_localization.landmark_localization:main',
        ],
    },
)
