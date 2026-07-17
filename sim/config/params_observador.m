%% PARÁMETROS Y GANANCIAS DEL OBSERVADOR DE ESTADO
% El observador se diseña con el modelo nominal Jeq_obs.

polo_obs_deseado = -3200;    % [rad/s] Dos polos coincidentes
polo_suave=-1800;            % [rad/s] Tres polos coincidentes

%% Observador REDUCIDO
%K_eth_num = 2*abs(polo_obs_deseado);   % [rad/s]
%K_ew_num  = abs(polo_obs_deseado)^2;   % [rad/s^2]

%% Observador con accion INTEGRAL
K_eth_num = 3 * abs(polo_suave);
K_ew_num  = 3 * abs(polo_suave)^2;
K_ei_num  = abs(polo_suave)^3;

assert(polo_obs_deseado < 0, ...
    'El polo del observador debe estar en el semiplano izquierdo.');
