from setuptools import find_packages, setup

package_name = 'demo_pkg'

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
    maintainer='Felipe Rivas',
    maintainer_email='rivascf@gmail.com',
    description='Paquete de ejemplo de ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mynodo = demo_pkg.MyNodo:main',
            'mysegnodo = demo_pkg.MySubscriptor:main',
            'paramsrv = demo_pkg.MyParameters:main'
        ],
    },
)
