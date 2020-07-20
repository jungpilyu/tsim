import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    ld = launch.LaunchDescription()
    ld.add_action(launch.actions.SetLaunchConfiguration('launch-prefix', 'xterm -e'))

    # ld.add_action(launch.actions.ExecuteProcess(
    #     cmd = ['ros2 run --prefix "xterm -e" tsim commander --ros-args -p key_in_period:=2.0'],
    #     name = 'commander',
    #     additional_env={'PYTHONUNBUFFERED': '1'},
    #     shell = True,
    #     output = 'screen',
    # ))

    parameter_path = join(get_package_share_directory('tsim'), 'param.yaml')

    ld.add_action(Node(
        package ='tsim',
        executable ='commander',
        name ='commander',
        output ='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        prefix = 'xterm -e ',#prefix = 'xterm -e python3 -m pdb',
        parameters = [{'key_in_period': 1.0}]
    ))
    ld.add_action(Node(
        package='tsim',
        executable='motors',
        name='motors',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        prefix = 'xterm -e',
        parameters = [parameter_path]
    ))
    ld.add_action(Node(
        package='tsim',
        executable='locator',
        name='locator',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        prefix = 'xterm -e'
    ))
    ld.add_action(Node(
        package='tsim',
        executable='requester',
        name='requester',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters = [parameter_path]
    ))
    ld.add_action(Node(
        package='tsim',
        executable='autopilot',
        name='autopilot',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        prefix = 'xterm -e'
    ))


    return ld

if __name__ == '__main__':
    
    ld = generate_launch_description()

    print('Starting introspection of launch description...\n')
    print(launch.LaunchIntrospector().format_launch_description(ld))
    print('\nStarting launch of launch description...\n')

    #ls = LaunchService(debug=True)
    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run(shutdown_when_idle = False)
