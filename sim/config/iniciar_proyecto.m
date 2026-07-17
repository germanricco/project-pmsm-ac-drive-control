% =========================================================================
% Inicio limpio del proyecto
% Use este archivo cuando quiera borrar resultados anteriores y cargar la
% configuración nominal completa.
% =========================================================================

clearvars;
close all;
clc;

carpeta_inicio = fileparts(mfilename('fullpath'));
run(fullfile(carpeta_inicio, 'init_sistema_completo.m'));
