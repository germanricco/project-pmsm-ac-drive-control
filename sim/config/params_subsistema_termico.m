%% PARÁMETROS DEL SUBSISTEMA TÉRMICO
% No usar clear, clc ni close all dentro de archivos de parámetros.

Cts     = 0.818;     % [J/°C] Capacitancia térmica equivalente
Rts_amb = 146.7;     % [°C/W] Resistencia térmica estator-ambiente

tau_ts_amb = Rts_amb*Cts;  % [s] Constante de tiempo térmica

% Temperaturas
Tamb_min = -15.0;    % [°C]
Tamb_max =  40.0;    % [°C]
Tamb     = Tamb_max; % [°C] Condición inicial y caso más exigente
Ts_max   = 115.0;    % [°C]

% Resistencia estatórica dependiente de la temperatura
Rs_REF   = 1.02;     % [ohm/fase] A Ts_REF
Ts_REF   = 20.0;     % [°C]
alpha_Cu = 3.9e-3;   % [1/°C]
