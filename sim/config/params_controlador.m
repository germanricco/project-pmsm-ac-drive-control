%% PARÁMETROS Y GANANCIAS DEL CONTROLADOR
% Las ganancias se diseñan con los parámetros nominales congelados
% Jeq_ctrl, beq_ctrl, etc. No deben recalcularse al variar la planta.

% -------------------------------------------------------------------------
% Lazos proporcionales de corriente
% -------------------------------------------------------------------------
p_c = -5000;                 % [rad/s] Polo deseado de los lazos de corriente

R_q_num = -p_c*L_q;          % [ohm]
R_d_num = -p_c*L_d;          % [ohm]
R_0_num = -p_c*L_l;          % [ohm]

% -------------------------------------------------------------------------
% Controlador PID de movimiento
% -------------------------------------------------------------------------
wn_pid   = 800;              % [rad/s]
zeta_pid = 0.75;             % [-]
n_pid    = 2*zeta_pid + 1;   % n = 2.5

ba_num   = Jeq_ctrl*n_pid*wn_pid;
Ksa_num  = Jeq_ctrl*n_pid*wn_pid^2;
Ksia_num = Jeq_ctrl*wn_pid^3;

Torque_max = 45;

assert(p_c < 0, 'El polo de corriente p_c debe estar en el semiplano izquierdo.');
assert(wn_pid > 0 && zeta_pid > 0, ...
    'wn_pid y zeta_pid deben ser positivos.');
