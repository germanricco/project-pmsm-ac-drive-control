%% Script de Validación Automatizado: Subsistema Electromagnético

% Instanciar el plotter apuntando al directorio de imágenes del repositorio
plotter = SignalPlotter("docs/img/");
tiempo_sim = 2.0; % [s] Tiempo total de simulación para capturar bien el paso en t = 1s

% =========================================================================
% EJECUCIÓN - PRUEBA 1: Inyección en el Eje Directo (Magnetización)
% =========================================================================
disp('Iniciando Prueba 1: Validación del Eje Directo (d)...');

% 1. Configuración de Entradas y Estímulos
theta_m_test = 0;       % [rad] Rotor bloqueado y alineado al origen
w_m_test = 0;           % [rad/s] Velocidad mecánica cero
T_s_test = 40;          % [°C] Temperatura estática para fijar Rs

% Vector espacial a 0° eléctricos (Escalón en t = 1s)
va_amp = 10;            % [V]
vb_amp = -5;            % [V]
vc_amp = -5;            % [V]

% 2. Ejecutar Simulación
out1 = sim('validacion_subsistema_electromagnetico', ...
           'MaxStep', '0.001', 'StopTime', num2str(tiempo_sim));

% 3. Extracción de Datos del Bus (Timeseries)
t1 = out1.motor_data.v_q.Time;
v_dq1 = [out1.motor_data.v_d.Data, out1.motor_data.v_q.Data];
i_dq1 = [out1.motor_data.i_d.Data, out1.motor_data.i_q.Data];
T_m1  = out1.motor_data.T_m.Data;

% 4. Generación de Gráficos - Prueba 1
[fig1_v, ax1_v] = plotter.plotTime(t1, v_dq1, ...
    'Title', 'Prueba 1 (Eje d): Tensiones de Control post-Park', ...
    'XLabel', 'Tiempo [s]', 'YLabel', 'Tensión [V]');
legend(ax1_v, 'v_d(t)', 'v_q(t)', 'Location', 'best');
plotter.export(fig1_v, 'fig_p1_tensiones_dq.svg');

[fig1_i, ax1_i] = plotter.plotTime(t1, i_dq1, ...
    'Title', 'Prueba 1 (Eje d): Transitorio RL de Corrientes', ...
    'XLabel', 'Tiempo [s]', 'YLabel', 'Corriente [A]');
legend(ax1_i, 'i_d(t) [Magnetización]', 'i_q(t) [Torque]', 'Location', 'best');
plotter.export(fig1_i, 'fig_p1_corrientes_dq.svg');

[fig1_t, ax1_t] = plotter.plotTime(t1, T_m1, ...
    'Title', 'Prueba 1 (Eje d): Torque Electromagnético Nulo', ...
    'XLabel', 'Tiempo [s]', 'YLabel', 'Torque [N·m]');
plotter.export(fig1_t, 'fig_p1_torque_nulo.svg');


% =========================================================================
% EJECUCIÓN - PRUEBA 2: Inyección en el Eje en Cuadratura (Torque)
% =========================================================================
disp('Iniciando Prueba 2: Validación del Eje en Cuadratura (q)...');

% 1. Configuración de Entradas y Estímulos
theta_m_test = 0;       % [rad] Rotor bloqueado
w_m_test = 0;           % [rad/s] Velocidad mecánica cero
T_s_test = 40;          % [°C] Mantenemos la temperatura para comparar Rs directamente

% Vector espacial a 90° eléctricos (Escalón en t = 1s)
va_amp = 0;             % [V]
vb_amp = 10 * (sqrt(3)/2); % [V] Aprox 8.66V
vc_amp = -10 * (sqrt(3)/2);% [V] Aprox -8.66V

% 2. Ejecutar Simulación
out2 = sim('validacion_subsistema_electromagnetico', ...
           'MaxStep', '0.001', 'StopTime', num2str(tiempo_sim));

% 3. Extracción de Datos del Bus (Timeseries)
t2 = out2.motor_data.v_q.Time;
v_dq2 = [out2.motor_data.v_d.Data, out2.motor_data.v_q.Data];
i_dq2 = [out2.motor_data.i_d.Data, out2.motor_data.i_q.Data];
T_m2  = out2.motor_data.T_m.Data;

% 4. Generación de Gráficos - Prueba 2
[fig2_v, ax2_v] = plotter.plotTime(t2, v_dq2, ...
    'Title', 'Prueba 2 (Eje q): Tensiones de Control post-Park', ...
    'XLabel', 'Tiempo [s]', 'YLabel', 'Tensión [V]');
legend(ax2_v, 'v_d(t)', 'v_q(t)', 'Location', 'best');
plotter.export(fig2_v, 'fig_p2_tensiones_dq.svg');

[fig2_i, ax2_i] = plotter.plotTime(t2, i_dq2, ...
    'Title', 'Prueba 2 (Eje q): Transitorio RL de Corrientes', ...
    'XLabel', 'Tiempo [s]', 'YLabel', 'Corriente [A]');
legend(ax2_i, 'i_d(t) [Magnetización]', 'i_q(t) [Torque]', 'Location', 'best');
plotter.export(fig2_i, 'fig_p2_corrientes_dq.svg');

[fig2_t, ax2_t] = plotter.plotTime(t2, T_m2, ...
    'Title', 'Prueba 2 (Eje q): Torque Electromagnético Activo', ...
    'XLabel', 'Tiempo [s]', 'YLabel', 'Torque [N·m]');
plotter.export(fig2_t, 'fig_p2_torque_activo.svg');

disp('Ensayo y validación electromagnética finalizada con éxito. Gráficos vectoriales SVG exportados.');