%% Script de Validación: Subsistema Mecánico
% =========================================================
% 1. VALIDACIÓN CASO NOMINAL (Masa estática)
% =========================================================

% Ejecutamos la simulación para el caso base
out = sim('validacion_subsistema_mecanico');

% Extraer datos nominales (este será nuestro vector de tiempo maestro)
tiempo_base = out.sim_out.time;
theta_base  = out.sim_out.signals.values;

% Instanciar el plotter apuntando al directorio de imágenes
plotter = SignalPlotter("docs/img/");

% Graficar y exportar el caso nominal
[fig1, ax1] = plotter.plotTime(tiempo_base, theta_base, ...
    'Title', 'Respuesta Libre del Péndulo (Condición Inicial: \pi/2)', ...
    'XLabel', 'Tiempo [s]', ...
    'YLabel', '\theta_m(t) [rad]');
ylim(ax1, [-2 2]); % Ajuste estético opcional
plotter.export(fig1, 'fig_respuesta_libre_mecanica.svg');


% =========================================================
% 2. EVALUACIÓN DE ROBUSTEZ (Barrido paramétrico de m_l)
% =========================================================
ml_vector = [0, 0.5, 1.0, 1.5]; % [kg] Valores a iterar

% Preasignar matriz de resultados usando la longitud del tiempo maestro
Y_matriz = zeros(length(tiempo_base), length(ml_vector));

for i = 1:length(ml_vector)
    % A. Actualizar valor en el Workspace
    ml_nom = ml_vector(i);
    
    % B. Recalcular las dependencias físicas
    Jl_nom = (m*Lcm^2 + Jcm) + ml_nom*Ll^2;
    Jeq_nom = Jm + Jl_nom/r^2;
    kg_eq_nom = g*(m*Lcm + ml_nom*Ll)/r;
    
    % C. Ejecutar simulación
    out_sweep = sim('validacion_subsistema_mecanico', 'MaxStep', '0.01'); 
    
    t_crudo = out_sweep.sim_out.time;
    theta_crudo = out_sweep.sim_out.signals.values;
    
    % D. Interpolar
    Y_matriz(:, i) = interp1(t_crudo, theta_crudo, tiempo_base, 'linear', 'extrap');
end

% Graficar el barrido paramétrico y exportar
[fig2, ax2] = plotter.plotTimeSweep(tiempo_base, Y_matriz, ml_vector, 'm_l', ...
    'Title', 'Respuesta Libre para distintos valores de Carga Util');
plotter.export(fig2, 'fig_barrido_masa_mecanica.svg');