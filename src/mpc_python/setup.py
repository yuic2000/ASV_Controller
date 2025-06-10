from setuptools import find_packages, setup

package_name = 'mpc_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['mpc_python/config/mpc.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yui',
    maintainer_email='yuic@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_cmd_pub = mpc_python.communication.mpc_cmd_pub:main',
            'gtodom_sub = mpc_python.communication.gtodom_sub:main',
            'main = mpc_python.main:main',
            'mpc = mpc_python.mpc:main',
        ],
    },
)
