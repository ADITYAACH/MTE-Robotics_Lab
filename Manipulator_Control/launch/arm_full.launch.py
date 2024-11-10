from launch import LaunchDescription 

from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess 

from launch.conditions import IfCondition 

from launch.event_handlers import OnProcessExit 

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution 

from launch_ros.actions import Node 

from launch_ros.substitutions import FindPackageShare 

import xacro  

def generate_launch_description(): 

    urdf_file = '/home/achanti/ros2_ws/src/urdf_tutorial/urdf/manipulator.urdf'    

    declared_arguments = [] 

    declared_arguments.append( 

        DeclareLaunchArgument( 

            "gui", 

            default_value="true", 

            description="Start RViz2 automatically with this launch file.", 

        ) 

    ) 

  

    gui = LaunchConfiguration("gui") 

  

    doc = xacro.parse(open(urdf_file))  

  

    xacro.process_doc(doc)  

  

    robot_description = {'robot_description': doc.toxml()}  

    robot_description_content = Command( 

        [ 

            PathJoinSubstitution([FindExecutable(name="xacro")]), 

            " ", 

            PathJoinSubstitution( 

                [ 

                    FindPackageShare("urdf_tutorial"), 

                    "manipulator.urdf", 

                ] 

            ), 

        ] 

    ) 

  

    robot_controllers = PathJoinSubstitution( 

        [ 

            FindPackageShare("urdf_tutorial"), 

            "control.yaml", 

        ] 

    ) 

  

    control_node = Node( 

        package="controller_manager", 

        executable="ros2_control_node", 

        parameters=[robot_controllers], 

        output="both", 

    ) 

    robot_state_pub_node = Node( 

        package="robot_state_publisher", 

        executable="robot_state_publisher", 

        output="both", 

        parameters=[robot_description], 

    ) 

    rviz_node = Node( 

        package="rviz2", 

        executable="rviz2", 

        name="rviz2", 

        output="log", 

        arguments=[], 

        condition=IfCondition(gui), 

    ) 

  

    joint_state_broadcaster_spawner = Node( 

        package="controller_manager", 

        executable="spawner", 

        arguments=["joint_state_broadcaster"], 

    ) 

  

    robot_controller_spawner = Node( 

        package="controller_manager", 

        executable="spawner", 

        arguments=["joint_trajectory_controller", "--param-file", robot_controllers], 

    ) 

  

    # Delay rviz start after `joint_state_broadcaster` 

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler( 

        event_handler=OnProcessExit( 

            target_action=joint_state_broadcaster_spawner, 

            on_exit=[rviz_node]   ) ) 

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler( 

        event_handler=OnProcessExit( 

            target_action=robot_controller_spawner, 

            on_exit=[joint_state_broadcaster_spawner],   )) 

    gazebo=ExecuteProcess(  

        cmd=["gazebo","-s","libgazebo_ros_factory.so",],  

        output="screen",   ) 

    gazebo_ros_node=Node(  

        package="gazebo_ros",  

        executable="spawn_entity.py",  

        arguments=["-entity","manipulator","-b","-file", urdf_file],  ) 

    nodes = [ 

        gazebo, 

        gazebo_ros_node, 

        control_node, 

        robot_state_pub_node, 

        robot_controller_spawner, 

        delay_rviz_after_joint_state_broadcaster_spawner, 

        delay_joint_state_broadcaster_after_robot_controller_spawner, 

    ] 

  

    return LaunchDescription(declared_arguments + nodes) 