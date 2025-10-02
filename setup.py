from setuptools import setup

package_name = 'draco_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngmo',
    maintainer_email='you@example.com',
    description='PointCloud2 TCP bridge with Draco (skeleton)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'encoder_server = draco_bridge.encoder_server:main',
            'decoder_client = draco_bridge.decoder_client:main',
        ],
    },
)
