from setuptools import setup

package_name = 'vlm_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='VLM bridge node for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_cli = vlm_bridge.vlm_cli:main',
            'vlm_bridge_node = vlm_bridge.vlm_bridge_node:main',
        ],
    },
)
