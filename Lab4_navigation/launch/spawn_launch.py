from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    '''    
    Example of AMCL (pure localization) for BB8
    '''

    sl = SimpleLauncher()
    
    sl.declare_arg('robot', 'bb8')
    robot = sl.arg('robot')
    robot_type = sl.py_eval("''.join(c for c in '", robot, "' if not c.isdigit())")
     
    sl.declare_arg('use_nav', False)
    use_nav = sl.arg('use_nav')
    
    nav_nodes = [('nav2_controller','controller_server'), ('nav2_planner','planner_server'), ('nav2_bt_navigator','bt_navigator')]
    node_names = []
    print(robot.describe())

    with sl.group(ns=robot):
                        
        sl.node('lab4_navigation', 'vel2joints.py', parameters = [{'static_tf': True}])
                       
        # generate description
        sl.robot_state_publisher('lab4_navigation', sl.name_join(robot_type, '.xacro'), 'urdf', xacro_args={'name': robot})

        with sl.group(unless_arg='use_nav'):
			# fire up slider publisher for cmd_vel
            cmd_file = sl.find('lab4_navigation', 'cmd_sliders.yaml')
            sl.node('slider_publisher', 'slider_publisher', name='cmd_vel_manual', arguments=[cmd_file])
        
        with sl.group(if_arg='use_nav'):
			# launch navigation nodes with remappings
            for pkg,executable in nav_nodes:
                robot_rad = sl.py_eval("'", robot_type, "'=='bb'and .27 or .16")
                configured_params = RewrittenYaml(source_file = sl.find('lab4_navigation', 'nav_param.yaml'), 
                                                  root_key = robot,
                                                  param_rewrites = {'robot_base_frame' : sl.py_eval("''.join(c for c in '", robot,"') + '/base_link'"),
                                                                    'global_frame' : sl.py_eval("''.join(c for c in '", robot,"') + '/odom'"),
                                                                    'topic': sl.py_eval("'/' + ''.join(c for c in '", robot,"') + '/scan'"),
                                                                    'robot_radius' : robot_rad,
                                                                    'default_bt_xml_filename': sl.find('nav2_bt_navigator','navigate_w_replanning_time.xml')
                                                                    },
                                                  convert_types = True) 
                sl.node(pkg, executable, name=executable,
                        parameters=[configured_params],
                        )
                node_names.append(executable)
            # run lifecycle manager just for navigation nodes
            sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager', output='screen', parameters=[{'autostart': True, 'node_names': node_names}])
              
    return sl.launch_description()
