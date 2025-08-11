from setuptools import setup

package_name = 'amr_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='AMR ROS2 Basic Communication Tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = amr_basics.talker:main',
            'listener = amr_basics.listener:main',
        ],
    },
)
