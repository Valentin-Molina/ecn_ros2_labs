from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('name', 'right_e0')
    sl.node('slider_publisher', 'slider_publisher', name = sl.arg('name'), arguments = [sl.find('move_joint', 'single_joint.yaml')])
    #sl.node('move_joint', 'move_joint', parameters = {'joint_name': 'right_e0'}, remappings = {'joint_setpoint': 'setpoint'}.items())
    
    sl.node('move_joint', 'move_joint', parameters = {'joint_name': sl.arg('name')}, remappings = {'joint_setpoint': 'setpoint'}.items())
    
    return sl.launch_description() 
