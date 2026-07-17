%% PARÁMETROS DEL SUBSISTEMA ELECTROMAGNÉTICO
% Este archivo solo define parámetros físicos y límites eléctricos.
% No usar clear, clc ni close all dentro de archivos de parámetros.

% Parámetros físicos de la PMSM
Pp       = 3;          % [-] Pares de polos
lambda_m = 0.016;      % [Wb] Flujo concatenado de los imanes
L_q      = 5.8e-3;     % [H] Inductancia del eje q
L_d      = 6.6e-3;     % [H] Inductancia del eje d
L_l      = 0.8e-3;     % [H] Inductancia de dispersión / eje cero

% Constantes electromagnéticas derivadas
K_t   = (3/2)*Pp*lambda_m;       % [N*m/A] Constante de torque con i_d = 0
K_v   = Pp*lambda_m;             % [V/(rad/s)] Constante de FEM
K_rel = (3/2)*Pp*(L_d - L_q);    % [N*m/A^2] Componente de reluctancia

% Límites de operación del motor
wm_max       = 691.15;   % [rad/s] Velocidad máxima/nominal del rotor
Is_rms_nom   = 0.4;      % [A rms] Corriente nominal de fase
Is_rms_max   = 2.0;      % [A rms] Corriente máxima indicada en la guía
Vsl_rms_nom  = 24.0;     % [V rms] Tensión nominal línea-línea
Vsl_rms_max  = 48.0;     % [V rms] Límite para el modulador no ideal
