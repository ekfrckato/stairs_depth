from setuptools import find_packages, setup

package_name = 'stairs_depth'

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
    maintainer='cae',
    maintainer_email='cae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'stairs_depth_orign = stairs_depth.stairs_depth_orign:main',
                            'stairs_motor_control = stairs_depth.stairs_motor_control:main',
                            'pub_diff_stairs_depth_imp = stairs_depth.pub_diff_stairs_depth_imp:main',
                            'diff_grapgh = stairs_depth.diff_graph:main',
                            'depth_imp_img = stairs_depth.depth_imp_img:main',
                            'keyboard = stairs_depth.press_keyboard:main'
        ],
    },
)
