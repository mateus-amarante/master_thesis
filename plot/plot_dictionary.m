function dict = plot_dictionary(lang)

if strcmp(lang,'en')
    dict.desired_leg = 'Desired';
    dict.actual_leg = 'Actual';
    dict.cable_orientation_title = 'Cable Orientation';
    dict.cable_rate_title = 'Cable Orientation Rate';
    dict.control_input_title = 'Control Input';
    dict.quad_position_title = 'Quadrotor Position';
    dict.quad_orientation_title = 'Quadrotor Orientation';
    dict.load_position_title= 'Load Position';
    dict.quad_linear_velocity_title = 'Quadrotor Linear Velocity';
    dict.quad_angular_velocity_euler_title  = 'Euler Angles Rate';
    dict.sliding_variables_title  = 'Sliding Variables';
    dict.time_label = 'Time [s]';
elseif strcmp(lang,'pt')
    dict.desired_leg = 'Desejado';
    dict.actual_leg = 'Realizado';
    dict.cable_orientation_title = 'Orientação do Cabo';
    dict.cable_rate_title = 'Taxa de Rotação do Cabo';
    dict.control_input_title = 'Entradas de Controle';
    dict.quad_position_title = 'Posição da Aeronave';
    dict.quad_o_title = 'Orientação da Aeronave';
    dict.load_position_title= 'Posição da Carga';
    dict.quad_linear_velocity_title = 'Velocidade Linear da Aeronave';
    dict.quad_angular_velocity_euler_title  = 'Velocidade Angular da Aeronave';
    dict.sliding_variables_title  = 'Variáveis Deslizantes';
    dict.time_label = 'Tempo [s]';
end

end

