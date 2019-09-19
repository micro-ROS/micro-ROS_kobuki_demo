from setuptools import find_packages
from setuptools import setup

package_name = 'odom_to_tf'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ralph Lange',
    author_email='ralph.lange@de.bosch.com',
    maintainer='Ralph Lange',
    maintainer_email='ralph.lange@de.bosch.com',
    keywords=['odom', 'tf'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = odom_to_tf.odom_to_tf:main',
            'circle_odom_publisher = odom_to_tf.circle_odom_publisher:main',
        ],
    },
)
