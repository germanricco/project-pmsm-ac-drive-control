%% Script de Validación Automatizado: Subsistema Térmico (Comparativa 40°C vs 20°C)
disp('Iniciando Validación del Subsistema Térmico (Comparativa)...');

% =========================================================================
% CONFIGURACIÓN BASE DEL ENSAYO (Corriente Nominal)
% =========================================================================
% Inyectamos la corriente pico equivalente a Is_nom = 0.4 Arms (0.4 * sqrt(2) = 0.565 A)
ia_test = 0.565;         % [A] Corriente Fase A 
ib_test = -0.2825;       % [A] Corriente Fase B 
ic_test = -0.2825;       % [A] Corriente Fase C 
tiempo_sim = 600;        % [s] Tiempo de simulación (5*tau)

% =========================================================================
% CASO 1: Operación en el Límite Ambiental (T_amb = 40 °C)
% =========================================================================
disp('Simulando Caso 1: T_amb = 40 °C...');
Tamb_test = 40;          % [°C] Temperatura ambiente y condición inicial
out_40 = sim('validacion_subsistema_termico', ...
             'StopTime', num2str(tiempo_sim), ...
             'MaxStep', '0.5');

% Extraemos datos del Caso 1
t_40   = out_40.thermal_data.T_s.Time;
T_s_40 = out_40.thermal_data.T_s.Data;
R_s_40 = out_40.thermal_data.R_s.Data;

% =========================================================================
% CASO 2: Operación en Ambiente Nominal (T_amb = 20 °C)
% =========================================================================
disp('Simulando Caso 2: T_amb = 20 °C...');
Tamb_test = 20;          % [°C] Temperatura ambiente y condición inicial
out_20 = sim('validacion_subsistema_termico', ...
             'StopTime', num2str(tiempo_sim), ...
             'MaxStep', '0.5');

% Extraemos datos del Caso 2
% (Asumimos vector de tiempo idéntico por ser paso controlado)
T_s_20 = out_20.thermal_data.T_s.Data;
R_s_20 = out_20.thermal_data.R_s.Data;

% =========================================================================
% GENERACIÓN DE GRÁFICOS COMPARATIVOS
% =========================================================================
plotter = SignalPlotter("docs/img/");

% Gráfico 1: Evolución de la Temperatura del Estator
% Agrupamos ambas señales en una matriz para plotearlas juntas
[fig1, ax1] = plotter.plotTime(t_40, [T_s_40, T_s_20], ...
    'Title', 'Calentamiento del Estator (Efecto Joule) - Comparativa Ambiental', ...
    'XLabel', 'Tiempo [s]', ...
    'YLabel', 'Temperatura T_s [°C]');

% Agregamos la línea de límite y la leyenda
yline(ax1, 115, '--r', 'T_{max} Límite (115°C)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
legend(ax1, 'T_s (T_{amb} = 40°C)', 'T_s (T_{amb} = 20°C)', 'Límite Operativo', 'Location', 'best');
plotter.export(fig1, 'fig_val_termica_Ts_comparativa.svg');

% Gráfico 2: Evolución de la Resistencia Dinámica
[fig2, ax2] = plotter.plotTime(t_40, [R_s_40, R_s_20], ...
    'Title', 'Aumento de la Resistencia por Temperatura - Comparativa', ...
    'XLabel', 'Tiempo [s]', ...
    'YLabel', 'Resistencia R_s [\Omega]');
legend(ax2, 'R_s (T_{amb} = 40°C)', 'R_s (T_{amb} = 20°C)', 'Location', 'best');
plotter.export(fig2, 'fig_val_termica_Rs_comparativa.svg');

disp('Validación térmica comparativa finalizada. Gráficos exportados con éxito.');