from setuptools import find_packages, setup

package_name = 'ros2_tutorial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 以下の行を追加 (launchファイルのインストール)
        ('share/' + package_name, ['launch/autonomous_move_turtlebot3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cassis-orange2',
    maintainer_email='c1531049@eng.kitakyu-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 以下の行を追加
            # 課題１
            # ros2_tutorial_pkgパッケージのpublisher.pyをノード名int64_publisherで実行
            'int64_publisher = ros2_tutorial_pkg.publisher:main',
            # ros2_tutorial_pkgパッケージのsubscriber.pyをノード名int64_subscriberで実行
            'int64_subscriber = ros2_tutorial_pkg.subscriber:main',

            # 課題２
            # ros2_tutorial_pkgパッケージのmove_turtlesim.pyをノード名move_turtlesimで実行
            'move_turtlesim = ros2_tutorial_pkg.move_turtlesim:main',

            # 課題４
            # ros2_tutorial_pkgパッケージのkeyboard_turtlebot3.pyをノード名keyboard_turtlebot3で実行
            'keyboard_turtlebot3 = ros2_tutorial_pkg.keyboard_turtlebot3:main',

            # 課題５
            # ros2_tutorial_pkgパッケージのautonomous_move_turtlebot3.pyをノード名autonomous_move_turtlebot3で実行
            'autonomous_move_turtlebot3 = ros2_tutorial_pkg.autonomous_move_turtlebot3:main',
        ],
    },
)
