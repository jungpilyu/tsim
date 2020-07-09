import launch
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()
    #ld.add_action(launch.actions.SetLaunchConfiguration('launch-prefix', 'xterm -e'))

    # ld.add_action(launch.actions.ExecuteProcess(
    #     cmd = ['ros2 run --prefix "xterm -e" tsim commander --ros-args -p key_in_period:=2.0'],
    #     name = 'commander',
    #     additional_env={'PYTHONUNBUFFERED': '1'},
    #     shell = True,
    #     output = 'screen',
    # ))
    ld.add_action(Node(
        package ='tsim',
        executable ='commander',
        name ='commander',
        output ='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        prefix = 'xterm -e',
        parameters = [{'key_in_period': 1.0}]
    ))
    ld.add_action(Node(
        package='tsim',
        executable='motors',
        name='motors',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        prefix = 'xterm -e',
        parameters = [{'param.yaml'}]
    ))
    ld.add_action(Node(
        package='tsim',
        executable='locator',
        name='locator',
    ))
    ld.add_action(Node(
        package='tsim',
        executable='requester',
        name='requester',
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
    ls.run()
