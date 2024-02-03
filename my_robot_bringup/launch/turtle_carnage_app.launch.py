from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():   # name has to be exactly same
    ld = LaunchDescription()

    turtlesim_node = Node(
        package = "turtlesim",     # pkg name from which exe has to be launched.
        executable="turtlesim_node"  # executable name (this name has to be same as the name from setup.py of that pkg)

    )
    
    turtle_spawner_node = Node(
        package = "catch_them_all",     # pkg name from which exe has to be launched.
        executable="turtle_spawner"  # executable name (this name has to be same as the name from setup.py of that pkg)

    )

    turtle_controller_node = Node(
        package = "catch_them_all",
        executable="turtle_controller"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)

    return ld