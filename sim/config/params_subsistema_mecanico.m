%% PARÁMETROS DEL SUBSISTEMA MECÁNICO
% Contiene los parámetros físicos, sus rangos y el modelo nominal usado
% para diseñar el controlador. Los parámetros reales de la planta se
% calculan por separado en actualizar_parametros_planta.m.

% Constantes y transmisión
g = 9.80665;    % [m/s^2] Aceleración de la gravedad
r = 120.0;      % [-] Relación de reducción total

% Fricción viscosa de la articulación
bl_nom = 0.10;          % [N*m/(rad/s)]
bl_min = 0.07;
bl_max = 0.13;

% Brazo manipulador
m   = 1.0;       % [kg] Masa del brazo
Lcm = 0.25;      % [m] Distancia del centro de masa a la articulación
Jcm = 0.0208;    % [kg*m^2] Inercia respecto del centro de masa
Ll  = 0.50;      % [m] Longitud hasta el extremo

% Carga útil
ml_nom = 0.0;    % [kg]
ml_min = 0.0;
ml_max = 1.5;

% Perturbación externa por contacto, en el eje de la carga
Tld_nom = 0.0;   % [N*m]
Tld_min = -5.0;
Tld_max =  5.0;

% Motor + caja, referidos al eje del motor
Jm = 14.0e-6;    % [kg*m^2]
bm = 15.0e-6;    % [N*m/(rad/s)]

% -------------------------------------------------------------------------
% MODELO NOMINAL: se usa para diseño de controlador, compensador y observador
% -------------------------------------------------------------------------
Jl_nom = (m*Lcm^2 + Jcm) + ml_nom*Ll^2;
kl_nom = m*Lcm + ml_nom*Ll;

Jeq_nom   = Jm + Jl_nom/r^2;
beq_nom   = bm + bl_nom/r^2;
kg_eq_nom = g*kl_nom/r;

% Extremos para análisis de sensibilidad
Jl_min = (m*Lcm^2 + Jcm) + ml_min*Ll^2;
Jl_max = (m*Lcm^2 + Jcm) + ml_max*Ll^2;

kl_min = m*Lcm + ml_min*Ll;
kl_max = m*Lcm + ml_max*Ll;

Jeq_min = Jm + Jl_min/r^2;
Jeq_max = Jm + Jl_max/r^2;
beq_min = bm + bl_min/r^2;
beq_max = bm + bl_max/r^2;

kg_eq_min = g*kl_min/r;
kg_eq_max = g*kl_max/r;

% Parámetros nominales congelados para el diseño
Jeq_ctrl   = Jeq_nom;
beq_ctrl   = beq_nom;
kl_ctrl    = kl_nom;
kg_eq_ctrl = g*kl_ctrl/r;

% Modelo nominal usado dentro del observador
Jeq_obs = Jeq_nom;
beq_obs = beq_nom;
