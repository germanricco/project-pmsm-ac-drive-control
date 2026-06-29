%% =========================================================
%  3. PARÁMETROS TÉRMICOS
% ==========================================================
Cts = 0.818;                       % [W/(°C/s)] Capacitancia Térmica
Rts_amb = 146.7;                   % [°C/W] Resistencia Térmica estator-ambiente
tau_ts_amb = Rts_amb*Cts;          % [s] Constante de Tiempo Térmica

Tamb_min = -15.0;                  % [°C]
Tamb_max = 40.0;                   % [°C]
Tamb = Tamb_max;                   % [°C] Valor inicial Recomendado

Ts_max = 115.0;                    % [°C] Temperatura de Estator Maxima

Rs_REF = 1.02;                     % [ohm] Resistencia por fase a Ts_REF
Ts_REF = 20.0;                     % [°C] Temperatura de estator de referencia
alpha_Cu = 3.9e-3;                 % [1/°C] Coeficiente de Aumento de Rs con Ts