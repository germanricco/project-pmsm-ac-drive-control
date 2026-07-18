%% params_sensores.m
% Inicialización de los parámetros, matrices y condiciones iniciales de los
% sensores no ideales.
%
% Este script define las variables utilizadas por los bloques State-Space
% del modelo "sensores_no_ideales_en_SS.slx":
%
%   Sensor de posición:      A_pos,   B_pos,   C_pos,   D_pos, X0_pos
%   Sensores de corriente:   A_iabc,  B_iabc,  C_iabc,  D_iabc,
%                            X0_ia, X0_ib, X0_ic
%   Sensor de temperatura:   A_T,     B_T,     C_T,     D_T,   X0_T
%
% Ejecutar este archivo antes de iniciar la simulación del modelo.
% No se utiliza "clear" para no eliminar otros parámetros ya cargados en
% el workspace del proyecto.

%% 1. Sensores de corriente de fase i_a, i_b e i_c
% Modelo pasa-bajos de segundo orden:
%
%                 wn_i^2
% G_i(s) = -------------------------
%          s^2 + 2*zeta_i*wn_i*s + wn_i^2

k_w_i     = 10000/6000;                 % Multiplicador del ancho de banda
wn_i_base = 6000;              % Frecuencia natural base [rad/s]
zeta_i    = 1;                 % Amortiguamiento crítico
wn_i      = k_w_i * wn_i_base; % Frecuencia natural efectiva [rad/s]

A_iabc = [0,       -1;
          wn_i^2,  -2*zeta_i*wn_i];

B_iabc = [1;
          0];

C_iabc = [0, 1];
D_iabc = 0;

% Con k_w_i = 1:
% A_iabc = [0  -1;  3.6e7  -1.2e4]

%% 2. Sensor de posición angular theta_m
% Modelo pasa-bajos de segundo orden:
%
%                  wn_pos^2
% G_pos(s) = -------------------------------
%            s^2 + 2*zeta_pos*wn_pos*s + wn_pos^2

k_w_pos     = 2.5;                     % Multiplicador del ancho de banda
wn_pos_base = 2000;                  % Frecuencia natural base [rad/s]
zeta_pos    = 1;                     % Amortiguamiento crítico
wn_pos      = k_w_pos * wn_pos_base; % Frecuencia natural efectiva [rad/s]

A_pos = [0,         -1;
         wn_pos^2,  -2*zeta_pos*wn_pos];

B_pos = [1;
         0];

C_pos = [0, 1];
D_pos = 0;

% Con k_w_pos = 1:
% A_pos = [0  -1;  4.0e6  -4.0e3]

%% 3. Sensor de temperatura T_s
% Modelo pasa-bajos de primer orden:
%
%                 1
% G_T(s) = ----------------
%          tau_T*s + 1

tau_T = 1; % Constante de tiempo del sensor [s]

A_T = -1/tau_T;
B_T = 1;
C_T = 1/tau_T;
D_T = 0;

% Valores numéricos:
% A_T = -0.05, B_T = 1, C_T = 0.05, D_T = 0

%% 4. Condiciones iniciales de los estados de los sensores
% Se calculan en estado estacionario para evitar un transitorio artificial
% de medición al comenzar la simulación:
%
%   0 = A*X0 + B*u0
%   y0 = C*X0 + D*u0
%
% Para la realización de segundo orden utilizada en los sensores de
% corriente y posición:
%
%   X0 = [(2*zeta/wn)*y0;
%         y0]
%
% Para la realización de primer orden utilizada en temperatura:
%
%   X0 = tau_T*y0

% Valores físicos iniciales de las variables medidas.
% Modificar estos valores si la planta comienza desde otro punto inicial.
i_a0     = 0;  % Corriente inicial de fase a [A]
i_b0     = 0;  % Corriente inicial de fase b [A]
i_c0     = 0;  % Corriente inicial de fase c [A]
theta_m0 = 0;  % Posición angular inicial del motor [rad]
%% Condiciones iniciales térmicas
T_amb0 = 40;       % Temperatura ambiente inicial [°C]
T_s0   = T_amb0;   % Estator inicialmente en equilibrio térmico

%% Estado inicial interno del sensor
X0_T = tau_T*T_s0;
% Estados iniciales para los tres bloques State-Space de corriente.
X0_ia = [(2*zeta_i/wn_i)*i_a0;
         i_a0];

X0_ib = [(2*zeta_i/wn_i)*i_b0;
         i_b0];

X0_ic = [(2*zeta_i/wn_i)*i_c0;
         i_c0];

% Estado inicial para el bloque State-Space del sensor de posición.
X0_pos = [(2*zeta_pos/wn_pos)*theta_m0;
          theta_m0];

% Estado inicial para el bloque State-Space del sensor de temperatura.
% Con T_s0 = 25 °C y tau_T = 20 s, X0_T = 500 y la salida inicial es:
% C_T*X0_T = (1/20)*500 = 25 °C.
X0_T = tau_T*T_s0;

%% 5. Comprobaciones básicas
assert(k_w_i > 0,     'k_w_i debe ser positivo.');
assert(k_w_pos > 0,   'k_w_pos debe ser positivo.');
assert(wn_i > 0,      'wn_i debe ser positivo.');
assert(wn_pos > 0,    'wn_pos debe ser positivo.');
assert(zeta_i > 0,    'zeta_i debe ser positivo.');
assert(zeta_pos > 0,  'zeta_pos debe ser positivo.');
assert(tau_T > 0,     'tau_T debe ser positivo.');

% Verifica que las salidas iniciales de los sensores coincidan con los
% valores físicos iniciales configurados.
tol_X0 = 1e-12;
assert(abs(C_iabc*X0_ia + D_iabc*i_a0 - i_a0) < tol_X0, ...
       'La condición inicial X0_ia no reproduce i_a0.');
assert(abs(C_iabc*X0_ib + D_iabc*i_b0 - i_b0) < tol_X0, ...
       'La condición inicial X0_ib no reproduce i_b0.');
assert(abs(C_iabc*X0_ic + D_iabc*i_c0 - i_c0) < tol_X0, ...
       'La condición inicial X0_ic no reproduce i_c0.');
assert(abs(C_pos*X0_pos + D_pos*theta_m0 - theta_m0) < tol_X0, ...
       'La condición inicial X0_pos no reproduce theta_m0.');
assert(abs(C_T*X0_T + D_T*T_s0 - T_s0) < tol_X0, ...
       'La condición inicial X0_T no reproduce T_s0.');
