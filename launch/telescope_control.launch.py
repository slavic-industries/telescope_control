import launch
import launch_ros.actions


def generate_launch_description():

  ld = launch.LaunchDescription()
  
  target_body_node = launch_ros.actions.Node(
    package='telescope_target_body',
    executable='target_body_node',
    name='target',
    namespace=''
  )

  ld.add_action(target_body_node)

  return ld