%% ACTUALIZAR PARÁMETROS REALES DE LA PLANTA
% Antes de ejecutar este script deben existir ml_planta y bl_planta.
% Ejemplo:
%   ml_planta = 1.5;
%   bl_planta = 0.07;
%   run("actualizar_parametros_planta.m")

assert(exist('ml_planta','var') == 1, ...
    'Debe definir ml_planta antes de ejecutar este script.');
assert(exist('bl_planta','var') == 1, ...
    'Debe definir bl_planta antes de ejecutar este script.');
assert(ml_planta >= ml_min && ml_planta <= ml_max, ...
    'ml_planta debe pertenecer al intervalo [ml_min, ml_max].');
assert(bl_planta >= bl_min && bl_planta <= bl_max, ...
    'bl_planta debe pertenecer al intervalo [bl_min, bl_max].');

Jl_planta = (m*Lcm^2 + Jcm) + ml_planta*Ll^2;
kl_planta = m*Lcm + ml_planta*Ll;

Jeq_planta   = Jm + Jl_planta/r^2;
beq_planta   = bm + bl_planta/r^2;
kg_eq_planta = g*kl_planta/r;

% Perturbación equivalente referida al eje del motor
Tld_eq_planta = Tld_nom/r;
