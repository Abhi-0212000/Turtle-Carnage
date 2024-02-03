from setuptools import find_packages, setup

package_name = 'catch_them_all'

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
    maintainer='abhi',
    maintainer_email='abhi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawn_client = catch_them_all.turtle_spawn_client:main",
            "catch_turtle = catch_them_all.catch_turtle:main",
            "turtle_life = catch_them_all.turtle_life:main",
            "turtle_path = catch_them_all.turtle_path:main",
            "turtle_controller = catch_them_all.turtle_controller:main",
            "turtle_spawner = catch_them_all.turtle_spawner:main"
        ],
    },
)
