from simple_launch import SimpleLauncher

def generate_launch_description():
  sl = SimpleLauncher()

  x = '0'
  y = '0'
  z = '0.1'
  yaw = '0'
  pitch = '3.14'
  roll = '0'

  sl.include('baxter_description', 'baxter_state_publisher_launch.py')
  sl.node('tf2_ros', 'static_transform_publisher', arguments = [x, y, z, yaw, pitch, roll, 'right_gripper', 'left_gripper_desired'])
  
  return sl.launch_description()
