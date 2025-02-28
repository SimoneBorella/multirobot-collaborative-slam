from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # General parameters

    task_launch_arg = DeclareLaunchArgument(
        'task', 
        description='Specifies the task to complete (skidpad, acceleration, autocross, trackdrive)'
    )

    x_init_launch_arg = DeclareLaunchArgument(
        'x_init',
        default_value='0.0',
        description='Initial x coordinate of the vehicle (m)'
    )

    y_init_launch_arg = DeclareLaunchArgument(
        'y_init',
        default_value='0.0',
        description='Initial y coordinate of the vehicle (m)'
    )

    yaw_init_launch_arg = DeclareLaunchArgument(
        'yaw_init', 
        default_value='0.0',
        description='Initial yaw of the vehicle (rad)'
    )

    v_x_init_launch_arg = DeclareLaunchArgument(
        'v_x_init',
        default_value='0.0',
        description='Initial x coordinate of the vehicle velocity (m/s)'
    )

    v_y_init_launch_arg = DeclareLaunchArgument(
        'v_y_init',
        default_value='0.0',
        description='Initial y coordinate of the vehicle velocity (m/s)'
    )

    yaw_r_init_launch_arg = DeclareLaunchArgument(
        'yaw_r_init', 
        default_value='0.0',
        description='Initial yaw rate of the vehicle (rad/s)'
    )

    # Path tracking parameters

    fitting_step_launch_arg = DeclareLaunchArgument(
        'fitting_step',
        default_value='0.1',
        description = ''
    )
    look_ahead_launch_arg = DeclareLaunchArgument(
        'look_ahead',
        default_value='3.0',
        description='Look ahead to determine the waypoint to follow (m)'
    )
    vel_ref_launch_arg = DeclareLaunchArgument(
        'vel_ref',
        default_value='5.0',
        description='Maximum velocity reference (m/s)'
    )
    lateral_acceleration_max_launch_arg = DeclareLaunchArgument(
        'lateral_acceleration_max',
        default_value='0.3',
        description='Maximum lateral acceleration (m/s^2)'
    )

    ts_mpc_launch_arg = DeclareLaunchArgument(
        'ts_mpc',
        default_value='0.05',
        description='Discretization time for mpc (s)'
    )
    n_sqp_launch_arg = DeclareLaunchArgument(
        'n_sqp',
        default_value='3',
        description = ''
    )
    n_reset_launch_arg = DeclareLaunchArgument(
        'n_reset',
        default_value='1000000000',
        description = ''
    )
    sqp_mixing_launch_arg = DeclareLaunchArgument(
        'sqp_mixing',
        default_value='0.9',
        description = ''
    )
    fitting_step_mpc_launch_arg = DeclareLaunchArgument(
        'fitting_step_mpc',
        default_value='0.05',
        description = ''
    )
    r_in_launch_arg = DeclareLaunchArgument(
        'r_in',
        default_value='0.4',
        description = ''
    )
    r_out_launch_arg = DeclareLaunchArgument(
        'r_out',
        default_value='0.4',
        description = ''
    )
    max_dist_proj_launch_arg = DeclareLaunchArgument(
        'max_dist_proj',
        default_value='3.0',
        description = ''
    )
    e_long_launch_arg = DeclareLaunchArgument(
        'e_long',
        default_value='1.25',
        description = ''
    )
    e_eps_launch_arg = DeclareLaunchArgument(
        'e_eps',
        default_value='0.95',
        description = ''
    )
    max_alpha_launch_arg = DeclareLaunchArgument(
        'max_alpha',
        default_value='0.15',
        description = ''
    )
    s_trust_region_launch_arg = DeclareLaunchArgument(
        's_trust_region',
        default_value='30.0',
        description = ''
    )
    vx_zero_launch_arg = DeclareLaunchArgument(
        'vx_zero',
        default_value='0.3',
        description = ''
    )
    cost_q_c_launch_arg = DeclareLaunchArgument(
        'cost_q_c',
        default_value='0.8',
        description = ''
    )
    cost_q_l_launch_arg = DeclareLaunchArgument(
        'cost_q_l',
        default_value='1000.0',
        description = ''
    )
    cost_q_vs_launch_arg = DeclareLaunchArgument(
        'cost_q_vs',
        default_value='30.0',
        description = ''
    )
    cost_q_mu_launch_arg = DeclareLaunchArgument(
        'cost_q_mu',
        default_value='0.1',
        description = ''
    )
    cost_q_r_launch_arg = DeclareLaunchArgument(
        'cost_q_r',
        default_value='1e-4',
        description = ''
    )
    cost_q_beta_launch_arg = DeclareLaunchArgument(
        'cost_q_beta',
        default_value='0.0',
        description = ''
    )
    cost_r_D_launch_arg = DeclareLaunchArgument(
        'cost_r_D',
        default_value='1e-1',
        description = ''
    )
    cost_r_delta_launch_arg = DeclareLaunchArgument(
        'cost_r_delta',
        default_value='1e-3',
        description = ''
    )
    cost_r_vs_launch_arg = DeclareLaunchArgument(
        'cost_r_vs',
        default_value='1e-1',
        description = ''
    )
    cost_r_dD_launch_arg = DeclareLaunchArgument(
        'cost_r_dD',
        default_value='1e-1',
        description = ''
    )
    cost_r_dDelta_launch_arg = DeclareLaunchArgument(
        'cost_r_dDelta',
        default_value='5e2',
        description = ''
    )
    cost_r_dVs_launch_arg = DeclareLaunchArgument(
        'cost_r_dVs',
        default_value='1e-4',
        description = ''
    )
    cost_q_c_N_mult_launch_arg = DeclareLaunchArgument(
        'cost_q_c_N_mult',
        default_value='1000.0',
        description = ''
    )
    cost_q_r_N_mult_launch_arg = DeclareLaunchArgument(
        'cost_q_r_N_mult',
        default_value='10.0',
        description = ''
    )
    cost_sc_quad_track_launch_arg = DeclareLaunchArgument(
        'cost_sc_quad_track',
        default_value='10000.0',
        description = ''
    )
    cost_sc_quad_tire_launch_arg = DeclareLaunchArgument(
        'cost_sc_quad_tire',
        default_value='6400.0',
        description = ''
    )
    cost_sc_quad_alpha_launch_arg = DeclareLaunchArgument(
        'cost_sc_quad_alpha',
        default_value='6400.0',
        description = ''
    )
    cost_sc_lin_track_launch_arg = DeclareLaunchArgument(
        'cost_sc_lin_track',
        default_value='100.0',
        description = ''
    )
    cost_sc_lin_tire_launch_arg = DeclareLaunchArgument(
        'cost_sc_lin_tire',
        default_value='80.0',
        description = ''
    )
    cost_sc_lin_alpha_launch_arg = DeclareLaunchArgument(
        'cost_sc_lin_alpha',
        default_value='80.0',
        description = ''
    )
    state_bound_X_l_launch_arg = DeclareLaunchArgument(
        'state_bound_X_l',
        default_value='-1000000000.0',
        description = ''
    )
    state_bound_Y_l_launch_arg = DeclareLaunchArgument(
        'state_bound_Y_l',
        default_value='-1000000000.0',
        description = ''
    )
    state_bound_phi_l_launch_arg = DeclareLaunchArgument(
        'state_bound_phi_l',
        default_value='-1000000000.0',
        description = ''
    )
    state_bound_vx_l_launch_arg = DeclareLaunchArgument(
        'state_bound_vx_l',
        default_value='0.0',
        description = ''
    )
    state_bound_vy_l_launch_arg = DeclareLaunchArgument(
        'state_bound_vy_l',
        default_value='-5.0',
        description = ''
    )
    state_bound_r_l_launch_arg = DeclareLaunchArgument(
        'state_bound_r_l',
        default_value='-2.0',
        description = ''
    )
    state_bound_s_l_launch_arg = DeclareLaunchArgument(
        'state_bound_s_l',
        default_value='-1.0',
        description = ''
    )
    state_bound_D_l_launch_arg = DeclareLaunchArgument(
        'state_bound_D_l',
        default_value='-0.0',
        description = ''
    )
    state_bound_delta_l_launch_arg = DeclareLaunchArgument(
        'state_bound_delta_l',
        default_value='-0.35',
        description = ''
    )
    state_bound_vs_l_launch_arg = DeclareLaunchArgument(
        'state_bound_vs_l',
        default_value='0.0',
        description = ''
    )
    state_bound_X_u_launch_arg = DeclareLaunchArgument(
        'state_bound_X_u',
        default_value='1000000000.0',
        description = ''
    )
    state_bound_Y_u_launch_arg = DeclareLaunchArgument(
        'state_bound_Y_u',
        default_value='1000000000.0',
        description = ''
    )
    state_bound_phi_u_launch_arg = DeclareLaunchArgument(
        'state_bound_phi_u',
        default_value='1000000000.0',
        description = ''
    )
    state_bound_vx_u_launch_arg = DeclareLaunchArgument(
        'state_bound_vx_u',
        default_value='15.0',
        description = ''
    )
    state_bound_vy_u_launch_arg = DeclareLaunchArgument(
        'state_bound_vy_u',
        default_value='5.0',
        description = ''
    )
    state_bound_r_u_launch_arg = DeclareLaunchArgument(
        'state_bound_r_u',
        default_value='2.0',
        description = ''
    )
    state_bound_s_u_launch_arg = DeclareLaunchArgument(
        'state_bound_s_u',
        default_value='1000000000.0',
        description = ''
    )
    state_bound_D_u_launch_arg = DeclareLaunchArgument(
        'state_bound_D_u',
        default_value='1.0',
        description = ''
    )
    state_bound_delta_u_launch_arg = DeclareLaunchArgument(
        'state_bound_delta_u',
        default_value='0.35',
        description = ''
    )
    state_bound_vs_u_launch_arg = DeclareLaunchArgument(
        'state_bound_vs_u',
        default_value='50.0',
        description = ''
    )
    input_bound_dD_l_launch_arg = DeclareLaunchArgument(
        'input_bound_dD_l',
        default_value='-20.0',
        description = ''
    )
    input_bound_dDelta_l_launch_arg = DeclareLaunchArgument(
        'input_bound_dDelta_l',
        default_value='-10.0',
        description = ''
    )
    input_bound_dVs_l_launch_arg = DeclareLaunchArgument(
        'input_bound_dVs_l',
        default_value='-50.0',
        description = ''
    )
    input_bound_dD_u_launch_arg = DeclareLaunchArgument(
        'input_bound_dD_u',
        default_value='20.0',
        description = ''
    )
    input_bound_dDelta_u_launch_arg = DeclareLaunchArgument(
        'input_bound_dDelta_u',
        default_value='10.0',
        description = ''
    )
    input_bound_dVs_u_launch_arg = DeclareLaunchArgument(
        'input_bound_dVs_u',
        default_value='50.0',
        description = ''
    )


    # Vehicle model parameters

    Cm1_launch_arg = DeclareLaunchArgument(
        'Cm1',
        default_value='5000.0',
        description='Motor model parameter'
    )
    Cm2_launch_arg = DeclareLaunchArgument(
        'Cm2',
        default_value='172.0',
        description='Motor model parameter'
    )
    Cr0_launch_arg = DeclareLaunchArgument(
        'Cr0',
        default_value='180.0',
        description='Rolling resistance parameter'
    )
    Cr2_launch_arg = DeclareLaunchArgument(
        'Cr2',
        default_value='0.7',
        description='Drag parameter'
    )
    B_launch_arg = DeclareLaunchArgument(
        'B',
        default_value='10.0',
        description='Pacejka tire model parameter'
    )
    C_launch_arg = DeclareLaunchArgument(
        'C',
        default_value='1.38',
        description='Pacejka tire model parameter'
    )
    D_launch_arg = DeclareLaunchArgument(
        'D',
        default_value='1500.0',
        description='Pacejka tire model parameter'
    )
    m_launch_arg = DeclareLaunchArgument(
        'm',
        default_value='190.0',
        description='Mass of the vehicle (kg)'
    )
    Iz_launch_arg = DeclareLaunchArgument(
        'Iz',
        default_value='110.0',
        description='Vehicle inertia (kg*m^2)'
    )
    lf_launch_arg = DeclareLaunchArgument(
        'lf',
        default_value='1.22',
        description='Front distance from cog (m)'
    )
    lr_launch_arg = DeclareLaunchArgument(
        'lr',
        default_value='1.22',
        description='Rear distance from cog (m)'
    )
    g_launch_arg = DeclareLaunchArgument(
        'g',
        default_value='9.81',
        description='Gravity constant (m/s^2)'
    )


    pure_pursuit_node = Node(
        package='path_tracking',
        executable='pure_pursuit',
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration('task'), "' == 'acceleration' or",
                    "'", LaunchConfiguration('task'), "' == 'skidpad' or",
                    "'", LaunchConfiguration('task'), "' == 'autocross' or",
                    "'", LaunchConfiguration('task'), "' == 'trackdrive'",
                ]
            )
        ),
        parameters=[{
            'task': LaunchConfiguration('task'),
            'fitting_step': LaunchConfiguration('fitting_step'),
            'ts_mpc': LaunchConfiguration('ts_mpc'),
            'look_ahead': LaunchConfiguration('look_ahead'),
            'vel_ref': LaunchConfiguration('vel_ref'),
            'lateral_acceleration_max': LaunchConfiguration('lateral_acceleration_max'),
            'wheelbase': PythonExpression(["float(", LaunchConfiguration('lf'), ") + float(", LaunchConfiguration('lr'), ")"]),
        }]
    )


    mpcc_node = Node(
        package='path_tracking',
        executable='mpcc',
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration('task'), "' == 'trackdrive'",
                ]
            )
        ),
        parameters=[{
            'task': LaunchConfiguration('task'),
            'x_init': LaunchConfiguration('x_init'),
            'y_init': LaunchConfiguration('y_init'),
            'yaw_init': LaunchConfiguration('yaw_init'),
            'v_x_init': LaunchConfiguration('v_x_init'),
            'v_y_init': LaunchConfiguration('v_y_init'),
            'yaw_r_init': LaunchConfiguration('yaw_r_init'),
            'ts_mpc': LaunchConfiguration('ts_mpc'),
            'n_sqp': LaunchConfiguration('n_sqp'),
            'n_reset': LaunchConfiguration('n_reset'),
            'sqp_mixing': LaunchConfiguration('sqp_mixing'),
            'fitting_step_mpc': LaunchConfiguration('fitting_step_mpc'),
            'r_in': LaunchConfiguration('r_in'),
            'r_out': LaunchConfiguration('r_out'),
            'max_dist_proj': LaunchConfiguration('max_dist_proj'),
            'e_long': LaunchConfiguration('e_long'),
            'e_eps': LaunchConfiguration('e_eps'),
            'max_alpha': LaunchConfiguration('max_alpha'),
            's_trust_region': LaunchConfiguration('s_trust_region'),
            'vx_zero': LaunchConfiguration('vx_zero'),
            'cost_q_c': LaunchConfiguration('cost_q_c'),
            'cost_q_l': LaunchConfiguration('cost_q_l'),
            'cost_q_vs': LaunchConfiguration('cost_q_vs'),
            'cost_q_mu': LaunchConfiguration('cost_q_mu'),
            'cost_q_r': LaunchConfiguration('cost_q_r'),
            'cost_q_beta': LaunchConfiguration('cost_q_beta'),
            'cost_r_D': LaunchConfiguration('cost_r_D'),
            'cost_r_delta': LaunchConfiguration('cost_r_delta'),
            'cost_r_vs': LaunchConfiguration('cost_r_vs'),
            'cost_r_dD': LaunchConfiguration('cost_r_dD'),
            'cost_r_dDelta': LaunchConfiguration('cost_r_dDelta'),
            'cost_r_dVs': LaunchConfiguration('cost_r_dVs'),
            'cost_q_c_N_mult': LaunchConfiguration('cost_q_c_N_mult'),
            'cost_q_r_N_mult': LaunchConfiguration('cost_q_r_N_mult'),
            'cost_sc_quad_track': LaunchConfiguration('cost_sc_quad_track'),
            'cost_sc_quad_tire': LaunchConfiguration('cost_sc_quad_tire'),
            'cost_sc_quad_alpha': LaunchConfiguration('cost_sc_quad_alpha'),
            'cost_sc_lin_track': LaunchConfiguration('cost_sc_lin_track'),
            'cost_sc_lin_tire': LaunchConfiguration('cost_sc_lin_tire'),
            'cost_sc_lin_alpha': LaunchConfiguration('cost_sc_lin_alpha'),
            'state_bound_X_l': LaunchConfiguration('state_bound_X_l'),
            'state_bound_Y_l': LaunchConfiguration('state_bound_Y_l'),
            'state_bound_phi_l': LaunchConfiguration('state_bound_phi_l'),
            'state_bound_vx_l': LaunchConfiguration('state_bound_vx_l'),
            'state_bound_vy_l': LaunchConfiguration('state_bound_vy_l'),
            'state_bound_r_l': LaunchConfiguration('state_bound_r_l'),
            'state_bound_s_l': LaunchConfiguration('state_bound_s_l'),
            'state_bound_D_l': LaunchConfiguration('state_bound_D_l'),
            'state_bound_delta_l': LaunchConfiguration('state_bound_delta_l'),
            'state_bound_vs_l': LaunchConfiguration('state_bound_vs_l'),
            'state_bound_X_u': LaunchConfiguration('state_bound_X_u'),
            'state_bound_Y_u': LaunchConfiguration('state_bound_Y_u'),
            'state_bound_phi_u': LaunchConfiguration('state_bound_phi_u'),
            'state_bound_vx_u': LaunchConfiguration('state_bound_vx_u'),
            'state_bound_vy_u': LaunchConfiguration('state_bound_vy_u'),
            'state_bound_r_u': LaunchConfiguration('state_bound_r_u'),
            'state_bound_s_u': LaunchConfiguration('state_bound_s_u'),
            'state_bound_D_u': LaunchConfiguration('state_bound_D_u'),
            'state_bound_delta_u': LaunchConfiguration('state_bound_delta_u'),
            'state_bound_vs_u': LaunchConfiguration('state_bound_vs_u'),
            'input_bound_dD_l': LaunchConfiguration('input_bound_dD_l'),
            'input_bound_dDelta_l': LaunchConfiguration('input_bound_dDelta_l'),
            'input_bound_dVs_l': LaunchConfiguration('input_bound_dVs_l'),
            'input_bound_dD_u': LaunchConfiguration('input_bound_dD_u'),
            'input_bound_dDelta_u': LaunchConfiguration('input_bound_dDelta_u'),
            'input_bound_dVs_u': LaunchConfiguration('input_bound_dVs_u'),
            'm': LaunchConfiguration('m'),
            'Iz': LaunchConfiguration('Iz'),
            'Cm1': LaunchConfiguration('Cm1'),
            'Cm2': LaunchConfiguration('Cm2'),
            'Cr0': LaunchConfiguration('Cr0'),
            'Cr2': LaunchConfiguration('Cr2'),
            'B': LaunchConfiguration('B'),
            'C': LaunchConfiguration('C'),
            'D': LaunchConfiguration('D'),
            'lf': LaunchConfiguration('lf'),
            'lr': LaunchConfiguration('lr'),
            'g': LaunchConfiguration('g'),
        }]
    )


    
    return LaunchDescription([
        # General parameters
        task_launch_arg,
        x_init_launch_arg,
        y_init_launch_arg,
        yaw_init_launch_arg,
        v_x_init_launch_arg,
        v_y_init_launch_arg,
        yaw_r_init_launch_arg,

        # Path tracking parameters
        fitting_step_launch_arg,
        look_ahead_launch_arg,
        vel_ref_launch_arg,
        lateral_acceleration_max_launch_arg,
        ts_mpc_launch_arg,
        n_sqp_launch_arg,
        n_reset_launch_arg,
        sqp_mixing_launch_arg,
        fitting_step_mpc_launch_arg,
        r_in_launch_arg,
        r_out_launch_arg,
        max_dist_proj_launch_arg,
        e_long_launch_arg,
        e_eps_launch_arg,
        max_alpha_launch_arg,
        s_trust_region_launch_arg,
        vx_zero_launch_arg,
        cost_q_c_launch_arg,
        cost_q_l_launch_arg,
        cost_q_vs_launch_arg,
        cost_q_mu_launch_arg,
        cost_q_r_launch_arg,
        cost_q_beta_launch_arg,
        cost_r_D_launch_arg,
        cost_r_delta_launch_arg,
        cost_r_vs_launch_arg,
        cost_r_dD_launch_arg,
        cost_r_dDelta_launch_arg,
        cost_r_dVs_launch_arg,
        cost_q_c_N_mult_launch_arg,
        cost_q_r_N_mult_launch_arg,
        cost_sc_quad_track_launch_arg,
        cost_sc_quad_tire_launch_arg,
        cost_sc_quad_alpha_launch_arg,
        cost_sc_lin_track_launch_arg,
        cost_sc_lin_tire_launch_arg,
        cost_sc_lin_alpha_launch_arg,
        state_bound_X_l_launch_arg,
        state_bound_Y_l_launch_arg,
        state_bound_phi_l_launch_arg,
        state_bound_vx_l_launch_arg,
        state_bound_vy_l_launch_arg,
        state_bound_r_l_launch_arg,
        state_bound_s_l_launch_arg,
        state_bound_D_l_launch_arg,
        state_bound_delta_l_launch_arg,
        state_bound_vs_l_launch_arg,
        state_bound_X_u_launch_arg,
        state_bound_Y_u_launch_arg,
        state_bound_phi_u_launch_arg,
        state_bound_vx_u_launch_arg,
        state_bound_vy_u_launch_arg,
        state_bound_r_u_launch_arg,
        state_bound_s_u_launch_arg,
        state_bound_D_u_launch_arg,
        state_bound_delta_u_launch_arg,
        state_bound_vs_u_launch_arg,
        input_bound_dD_l_launch_arg,
        input_bound_dDelta_l_launch_arg,
        input_bound_dVs_l_launch_arg,
        input_bound_dD_u_launch_arg,
        input_bound_dDelta_u_launch_arg,
        input_bound_dVs_u_launch_arg,

        # Vehicle model parameters
        m_launch_arg,
        Iz_launch_arg,
        Cm1_launch_arg,
        Cm2_launch_arg,
        Cr0_launch_arg,
        Cr2_launch_arg,
        B_launch_arg,
        C_launch_arg,
        D_launch_arg,
        lf_launch_arg,
        lr_launch_arg,
        g_launch_arg,

        # Nodes
        pure_pursuit_node,
        mpcc_node,
    ])