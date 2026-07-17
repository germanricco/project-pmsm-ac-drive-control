% =========================================================================
% Script Maestro de Inicialización: init_sistema_completo.m
% Carga todos los parámetros necesarios para ejecutar el modelo Simulink.
% No limpia el Workspace para que pueda reutilizarse desde distintos
% scripts de simulación y campañas de escenarios.
% =========================================================================

clc;

disp('--------------------------------------------------');
disp('Iniciando carga de parámetros del sistema completo...');
disp('--------------------------------------------------');

% Carpeta donde se encuentra este archivo. Permite ejecutar el init aunque
% la carpeta actual de MATLAB sea otra.
carpeta_config = fileparts(mfilename('fullpath'));

try
    % ---------------------------------------------------------------------
    % 1. Parámetros físicos
    % ---------------------------------------------------------------------
    disp('Cargando: params_subsistema_mecanico.m ...');
    run(fullfile(carpeta_config, 'params_subsistema_mecanico.m'));

    disp('Cargando: params_subsistema_electromagnetico.m ...');
    run(fullfile(carpeta_config, 'params_subsistema_electromagnetico.m'));

    disp('Cargando: params_subsistema_termico.m ...');
    run(fullfile(carpeta_config, 'params_subsistema_termico.m'));

    % ---------------------------------------------------------------------
    % 2. Planta que se simulará
    % ---------------------------------------------------------------------
    % Si el script que llama al init no definió otro caso, se usa la planta
    % nominal. Esto permite sobrescribir ml_planta y bl_planta antes de
    % ejecutar nuevamente actualizar_parametros_planta.m.
    if ~exist('ml_planta', 'var')
        ml_planta = ml_nom;
    end

    if ~exist('bl_planta', 'var')
        bl_planta = bl_nom;
    end

    disp('Calculando parámetros de la planta seleccionada ...');
    run(fullfile(carpeta_config, 'actualizar_parametros_planta.m'));

    % ---------------------------------------------------------------------
    % 3. Parámetros de diseño del controlador y del observador
    % ---------------------------------------------------------------------
    disp('Cargando: params_controlador.m ...');
    run(fullfile(carpeta_config, 'params_controlador.m'));

    disp('Cargando: params_observador.m ...');
    run(fullfile(carpeta_config, 'params_observador.m'));

    % ---------------------------------------------------------------------
    % 4. Parámetros generales de simulación
    % ---------------------------------------------------------------------
    disp('Cargando: params_simulacion.m ...');
    run(fullfile(carpeta_config, 'params_simulacion.m'));

    % ---------------------------------------------------------------------
    % 5. Verificación mínima del Workspace
    % ---------------------------------------------------------------------
    variables_requeridas = {
        'Pp', 'lambda_m', 'L_q', 'L_d', 'L_l', ...
        'Jeq_planta', 'beq_planta', 'kg_eq_planta', ...
        'Jeq_ctrl', 'beq_ctrl', 'kg_eq_ctrl', ...
        'Jeq_obs', 'beq_obs', ...
        'R_q_num', 'R_d_num', 'R_0_num', ...
        'ba_num', 'Ksa_num', 'Ksia_num', ...
        'K_eth_num', 'K_ew_num', ...
        'Cts', 'Rts_amb', 'Tamb', 'Rs_REF', 'alpha_Cu', ...
        't_stop'};

    variables_faltantes = {};

    for k = 1:numel(variables_requeridas)
        nombre = variables_requeridas{k};
        if ~exist(nombre, 'var')
            variables_faltantes{end+1} = nombre; %#ok<SAGROW>
        end
    end

    if ~isempty(variables_faltantes)
        error('Inicializacion:VariablesFaltantes', ...
            'Faltan variables requeridas: %s', ...
            strjoin(variables_faltantes, ', '));
    end

    disp('--------------------------------------------------');
    disp('¡ÉXITO! El sistema quedó listo para simular.');
    fprintf('Planta: ml = %.3f kg | bl = %.3f N*m/(rad/s)\n', ...
        ml_planta, bl_planta);
    fprintf('Jeq_planta = %.8g kg*m^2\n', Jeq_planta);
    fprintf('Jeq_ctrl   = %.8g kg*m^2\n', Jeq_ctrl);
    fprintf('PID: ba = %.8g | Ksa = %.8g | Ksia = %.8g\n', ...
        ba_num, Ksa_num, Ksia_num);
    fprintf('Observador: K_eth = %.8g | K_ew = %.8g\n', ...
        K_eth_num, K_ew_num);
    disp('--------------------------------------------------');

catch ME
    disp('--------------------------------------------------');
    fprintf(2, 'ERROR: Falló la inicialización del sistema.\n');
    fprintf(2, '%s\n', ME.message);
    disp('--------------------------------------------------');

    % Se vuelve a lanzar el error para impedir que se simule con un
    % Workspace incompleto o con valores antiguos.
    rethrow(ME);
end
