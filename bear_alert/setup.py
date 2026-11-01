from setuptools import setup

package_name = 'bear_alert'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai-student',
    maintainer_email='noreply@example.com',
    description='Detect bears in a video with YOLO and make CRANE+ V2 perform a warning pose.',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [
            'bear_detector = bear_alert.bear_detector:main',
            'bear_warning = bear_alert.bear_warning:main',
        ],
    },
)
