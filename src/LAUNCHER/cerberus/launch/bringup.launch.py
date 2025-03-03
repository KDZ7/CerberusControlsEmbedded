import os
import re
from relaymd.relay import Relay
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def addEnvironment(key, tuple=None):
    if tuple is not None and Relay.dict is not None:
        value = Relay.dict
        for item in tuple:
            value = value.get(item, None)
        current = os.environ.get(key, "")
        if current:
            os.environ[key] = current + os.pathsep + str(value)
        else:
            os.environ[key] = str(value)

def bindContext(context):
    if "pkg_robot_description" in Relay.dict and Relay.dict["pkg_robot_description"] is not None:
        Relay.dict["robot_description"] = Command([" xacro ", PathJoinSubstitution([FindPackageShare(Relay.dict["pkg_robot_description"]), Relay.dict["robot_description"]])])
    else:
        Relay.dict["robot_description"] = Command([" xacro ", Relay.dict["robot_description"]])
    
    regex_fetch_package = re.compile(r'\[([^\]]+)\]')

    if "controller_manager" in Relay.dict:
        setup_path = Relay.dict["controller_manager"]["setup_path"]
        if setup_path is not None:
            matches =  regex_fetch_package.findall(setup_path)
            for match in matches:
                setup_path = setup_path.replace("[" + match + "]", FindPackageShare(match).perform(context))
            Relay.dict["controller_manager"]["setup_path"] = setup_path

    if "environment" in Relay.dict:
        res_paths = Relay.dict["environment"]["res_paths"]
        matches =  regex_fetch_package.findall(res_paths)
        for match in matches:
            res_paths = res_paths.replace("[" + match + "]", FindPackageShare(match).perform(context))
        plug_paths = Relay.dict["environment"]["plug_paths"]
        matches =  regex_fetch_package.findall(plug_paths)
        for match in matches:
            plug_paths = plug_paths.replace("[" + match + "]", FindPackageShare(match).perform(context))
        lib_paths = Relay.dict["environment"]["lib_paths"]
        matches =  regex_fetch_package.findall(lib_paths)
        for match in matches:
            lib_paths = lib_paths.replace("[" + match + "]", FindPackageShare(match).perform(context))
        Relay.dict["environment"]["res_paths"] = res_paths
        Relay.dict["environment"]["plug_paths"] = plug_paths
        Relay.dict["environment"]["lib_paths"] = lib_paths

def exec(context):
    tasks = []
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=Relay.dict["namespace"],
        name="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(Relay.dict["robot_description"], value_type=str)}],
        output="screen",
        remappings=[("joint_states", "/csystem/joint_states")]
    )
    tasks.append(robot_state_publisher)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=Relay.dict["namespace"],
        name="controller_manager",
        parameters=[
            {
                "robot_description": ParameterValue(Relay.dict["robot_description"], value_type=str),
                "update_rate": Relay.dict["controller_manager"]["update_rate"]
            },
            Relay.dict["controller_manager"]["setup_path"]
        ],
        output="screen"
    )

    csystem_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=Relay.dict["namespace"],
        name="csystem_controller_spawner",
        arguments=["csystem_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
 
    iksolver = Node(
        package="iksolver",
        executable="iksolver",
        namespace=Relay.dict["namespace"],
        name="iksolver",
        parameters=[{
            "config_file_path": os.path.join(get_package_share_directory("iksolver"), "config", "ikconfig.yaml"),
            "urdf_content": ParameterValue(Relay.dict["robot_description"], value_type=str)
        }],
        output="screen",
        remappings=[
            ("~/waypoint", "/waypoint"),
            ("~/joint_states", "/csystem/joint_states"),
            ("~/solutions", "/solutions/in")
        ]
    )

    transistion_iksolver = ExecuteProcess(
        cmd=["sleep 2 && ros2 lifecycle set /iksolver configure && sleep 2 && ros2 lifecycle set /iksolver activate"],
        shell=True
    )

    iksolver_bridge = Node(
            package="iksolver_bridge",
            executable="iksolver_bridge",
            namespace=Relay.dict["namespace"],
            name="iksolver_bridge",
            parameters=[{"config_file_path": os.path.join(get_package_share_directory("iksolver_bridge"), "config", "iksolver_bridge_config.yaml")}],
            output="screen",
            remappings=[
                ("~/in",  "/solutions/in"),
                ("~/out", "/solutions/out")
            ]
        )
    
    test_iksolver = Node(
        package="test_iksolver",
        executable="target",
        namespace=Relay.dict["namespace"],
        name="target",
        parameters=[{"base_frame": "M"}],
        output="screen",
        remappings=[
            ("~/waypoint", "/waypoint"),
            ("~/target", "/clicked_point")
        ]
    )

    gmotion = Node(
        package="gmotion",
        executable="gmotion",
        namespace=Relay.dict["namespace"],
        name="gmotion",
        parameters=[{"config_file_path": os.path.join(get_package_share_directory("gmotion"), "config", "gmotion.yaml")}],
        output="screen",
        remappings=[
            ("~/command", "/command"),
            ("~/solutions", "/solutions/out"),
            ("~/in", "/csystem/joint_states"),
            ("~/out", "/csystem/joint_commands"),
            ("~/waypoint", "/waypoint")
        ]
    )

    transistion_gmotion = ExecuteProcess(
        cmd=["sleep 2 && ros2 lifecycle set /gmotion configure && sleep 2 && ros2 lifecycle set /gmotion activate"],
        shell=True
    )

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     namespace=Relay.dict["namespace"],
    #     name="rviz2",
    #     arguments=["-d", os.path.join(get_package_share_directory("cerberus"), "config", "rviz.yaml")],
    #     output="screen"
    # )


    event_handler = [
        RegisterEventHandler(OnProcessStart(target_action=robot_state_publisher, on_start=[controller_manager])),
        RegisterEventHandler(OnProcessStart(target_action=controller_manager, on_start=[csystem_controller_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=csystem_controller_spawner, on_start=[iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=iksolver, on_start=[transistion_iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=iksolver, on_start=[iksolver_bridge])),
        RegisterEventHandler(OnProcessStart(target_action=iksolver_bridge, on_start=[test_iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=test_iksolver, on_start=[gmotion])),
        RegisterEventHandler(OnProcessStart(target_action=gmotion, on_start=[transistion_gmotion])),
        # RegisterEventHandler(OnProcessStart(target_action=transistion_gmotion, on_start=[rviz]))
    ]
    
    tasks.extend(event_handler)
    return tasks

def generate_launch_description():
    cfgrelay = LaunchConfiguration("relay")
    _cfgrelay = DeclareLaunchArgument(
        "relay",
        default_value=PathJoinSubstitution([FindPackageShare("cerberus"), "config", "relay.yaml"]),
        description="""
            ros_domain_id: [ROS domain id] (default: 0)
            namespace: [namespace] (default: Null)
            pkg_robot_description: [package name of robot description file] (default: Null)
            robot_description: [path to robot description file]

            controller_manager:
                setup_path: [path to controller manager setup file]
                update_rate: [update rate of controller manager] (default: 100)

            environment:
                res_paths: [paths to the resources] ex: [ros_package1]/models/sdf:[ros_package2]/models/urdf:/absolute/path/models/urdf etc ..
                plug_paths: [paths to the plugins] ex: [ros_package1]/lib:[ros_package2]/lib:/absolute/path/lib etc ..
                lib_paths: [paths to the libraries] ex: [ros_package1]/lib:[ros_package2]/lib:/absolute/path/lib etc ..
            """
    )
    
    actions = []
    actions.append(_cfgrelay)
    actions.append(Relay.read(cfgrelay))
    actions.append(Relay.action(bindContext))
    actions.append(Relay.action(lambda _: addEnvironment("ROS_DOMAIN_ID", ("ros_domain_id",))))
    actions.append(Relay.action(lambda _: addEnvironment("LD_LIBRARY_PATH", ("environment", "lib_paths"))))
    actions.append(ExecuteProcess(cmd=["bash", "-c", "printenv | grep -E 'ROS_DOMAIN_ID|GZ_SIM_RESOURCE_PATH|GZ_SIM_SYSTEM_PLUGIN_PATH|LD_LIBRARY_PATH|MESA_D3D12_DEFAULT_ADAPTER_NAME'"], output="screen"))
    actions.append(Relay.action(exec))
    return LaunchDescription([_cfgrelay, *actions])
