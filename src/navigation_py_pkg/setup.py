from setuptools import setup

package_name = 'navigation_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='codetc',
    maintainer_email='abaran7@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ 
        'console_scripts': [
            "simple_commander = navigation_py_pkg.simple_commander:main"
        ],
    },
)
