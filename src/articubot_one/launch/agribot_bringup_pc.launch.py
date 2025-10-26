# launch/agribot_bringup.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_serial_motor_demo = get_package_share_directory('serial_motor_demo') # Assuming your bridge node is here
    # If articubot_one holds the URDF/Lidar launch
    pkg_articubot_one = get_package_share_directory('articubot_one') 

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port', default='/dev/ttyUSB0') # Adjust as needed
    motor_serial_port = LaunchConfiguration('motor_serial_port', default='/dev/ttyACM0') # Adjust as needed
    baud_rate = LaunchConfiguration('baud_rate', default='57600')
    wheel_base = LaunchConfiguration('wheel_base', default='0.2') # ADJUST THIS
    
    # --- Arguments ---
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_lidar_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar')
    declare_motor_port_arg = DeclareLaunchArgument(
        'motor_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Motor Driver Arduino')
    declare_baud_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='57600',
        description='Baud rate for Motor Driver Arduino')
    declare_wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.2', # ADJUST THIS
        description='Distance between robot wheels (meters)')

    # --- Nodes and Includes ---

    # 1. Robot State Publisher (Publishes TF from URDF)
    # Re-use rsp.launch.py from articubot_one if it works for you
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_articubot_one, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Motor Driver Node (Connects to Arduino)
    motor_driver_node = Node(
        package='serial_motor_demo',
        executable='driver',
        name='serial_motor_driver',
        parameters=[{
            'serial_port': motor_serial_port,
            'baud_rate': baud_rate,
            'encoder_cpr': 1,  # Dummy value
            'loop_rate': 30    # Matches firmware
            }],
        output='screen'
    )

    # 3. Twist to Motor Command Bridge
    twist_to_motor_node = Node(
        package='serial_motor_demo',
        executable='twist_to_motor',
        name='twist_to_motor_bridge',
        parameters=[{'wheel_base': wheel_base}], # Pass the wheel_base
        output='screen',
        # Remap cmd_vel if using twist_mux or other sources
        # remappings=[('/cmd_vel', '/cmd_vel_raw')] 
    )

    # 4. RPLidar Node
    # Option 1: Basic Node definition
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'serial_port': lidar_serial_port, 
                     'serial_baudrate': 115200,  # Baud rate for C1 is usually 115200 or 256000
                     'frame_id': 'laser_frame', # Must match your URDF link
                     'inverted': False, 
                     'angle_compensate': True}],
        output='screen'
    )
    # Option 2: Include rplidar launch file if articubot_one has one you like
    # rplidar_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_articubot_one, 'launch', 'rplidar.launch.py') 
    #    ),
    #    launch_arguments={'serial_port': lidar_serial_port, 
    #                      'frame_id': 'laser_frame'}.items() # Adjust args as needed
    #)

    # 5. IMU Driver Node 
    imu_driver_node = Node(
        package='mpu6050_driver',          # Use the new package name
        executable='mpu6050_node',         # Use the executable name from setup.py
        name='imu_driver_node',
        output='screen',
        parameters=[{'frame_id': 'imu_link'}] # Example if the node supports setting frame_id via param
                                              # Otherwise, ensure it's hardcoded as 'imu_link' in the script
    )


    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_lidar_port_arg,
        declare_motor_port_arg,
        declare_baud_arg,
        declare_wheel_base_arg,

        rsp_launch,            # Start robot_state_publisher
        motor_driver_node,     # Start Arduino communication
        twist_to_motor_node,   # Start Twist bridge
        lidar_node,            # Start RPLidar
        # rplidar_launch, # Use this instead of lidar_node if including launch file
    ])