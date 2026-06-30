% =========================================================================
% Script Maestro de Inicialización: init_sistema_completo.m
% Carga los parámetros de todos los subsistemas en el Workspace base.
% =========================================================================

% Limpiar el entorno
clear; 
clc;

disp('--------------------------------------------------');
disp('Iniciando carga de parámetros del sistema acoplado...');
disp('--------------------------------------------------');

try
    % Cargar parámetros del subsistema mecánico
    disp('Cargando: params_subsistema_mecanico.m ...');
    run('params_subsistema_mecanico.m');
    
    % Cargar parámetros del subsistema electromagnético
    disp('Cargando: params_subsistema_electromagnetico.m ...');
    run('params_subsistema_electromagnetico.m');
    
    % Cargar parámetros del subsistema térmico
    disp('Cargando: params_subsistema_termico.m ...');
    run('params_subsistema_termico.m');
    
    disp('--------------------------------------------------');
    disp('¡ÉXITO! Todos los parámetros han sido cargados.');
    disp('--------------------------------------------------');
    
catch ME
    % Manejo de errores en caso de que falte un archivo o tenga un error de sintaxis
    disp('--------------------------------------------------');
    warning('ERROR: Falló la carga de parámetros.');
    disp(ME.message);
    disp('--------------------------------------------------');
end