from setuptools import find_packages, setup

package_name = 'pendulum_nodes'

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
    maintainer='termvato',
    maintainer_email='inasaridzevato@gmail.com',
    description='implementing a periodic joint input for a disturbance effect that a controller will try to fix',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pendulum_pub = pendulum_nodes.pendulum_pub:main',
        ],
    },
)
