from setuptools import setup, find_packages

package_name = 'adapt_roucomp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # Automatically find packages
    include_package_data=True,  # Include package data specified in MANIFEST.in
    package_data={
        package_name: ['config/*.txt'],  # Include all the .txt file
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/map.txt', 'config/map_cardinal.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Ibrahim Aldabbagh",
    maintainer_email="eng.ibrahim.aldabbagh@gmail.com",
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route = adapt_roucomp.routemodule5:main',
            'route_cardinal = adapt_roucomp.route_cardinal:main',
        ],
    },
)

