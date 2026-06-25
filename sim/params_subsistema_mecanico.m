% =========================================================
%  1. PARÁMETROS MECÁNICOS
% =========================================================
g = 9.80665;                            % [m/s^2] Gravedad
r = 120.0;                              % Relación de reducción total

% Coeficiente de friccion viscosa en la articulacion
bl_nom = 0.1;                           % [N*m/(rad/s)]
bl_min = 0.1 - 0.03;
bl_max = 0.1 + 0.03;

% Articulacion
m = 1.0;                                % [kg] masa del brazo
Lcm = 0.25;                             % [m] distancia al CM del brazo
Jcm = 0.0208;                           % [kg*m^2] inercia respecto al CM
Ll = 0.50;                              % [m] longitud total del brazo

% Carga útil
ml_nom = 0.0;                           % [kg]
ml_min = 0.0;
ml_max = 1.5;

% Momento de Inercia Total con Carga
Jl_nom = (m*Lcm^2 + Jcm) + ml_nom*Ll^2; % [kg*m^2]
Jl_min = (m*Lcm^2 + Jcm) + ml_min*Ll^2;
Jl_max = (m*Lcm^2 + Jcm) + ml_max*Ll^2;

% Coeficiente Gravitacional
kl_nom = m*Lcm + ml_nom*Ll;             % [kg*m]
kl_min = m*Lcm + ml_min*Ll;
kl_max = m*Lcm + ml_max*Ll;

% Perturbación externa por contacto aplicada en la carga
Tld_nom = 0;                            % [N*m]
Tld_min = -5.0;
Tld_max = 5.0;

Tld_sim_max = 6.28;                     % [N*m] valor pedido para simulación del punto 5.1.6

% Motor + caja, referido al eje del motor
Jm = 14.0e-6;                           % [kg*m^2] Momento de Inercia de Motor + Caja Reductora
bm = 15.0e-6;                           % [N*m/(rad/s)] Coeficiente de Friccion viscosa Motor + Caja Reductora

% =========================================================
%  2. PARÁMETROS EQUIVALENTES REFERIDOS AL EJE DEL MOTOR
% =========================================================
% Inercia Equivalente
Jeq_nom = Jm + Jl_nom/r^2;              % [kg*m^2]
Jeq_min = Jm + Jl_min/r^2;
Jeq_max = Jm + Jl_max/r^2;

% Friccion Viscosa Equivalente
beq_nom = bm + bl_nom/r^2;              % [N*m/(rad/s)]
beq_min = bm + bl_min/r^2;
beq_max = bm + bl_max/r^2;

% Coeficiente de Torque Gravitacional Equivalente Referido al Eje del Motor
kg_eq_nom = g*kl_nom/r;                 % [N*m]
kg_eq_min = g*kl_min/r;
kg_eq_max = g*kl_max/r;

% Perturbacion Externa Equivalente
Tld_eq_nom = Tld_nom/r;                 % [N*m]
Tld_eq_min = Tld_min/r;
Tld_eq_max = Tld_max/r;                 

Tld_sim_eq_max = Tld_sim_max/r;         % [N*m] Valor Pedido para simulacion del punto 5.1.6

