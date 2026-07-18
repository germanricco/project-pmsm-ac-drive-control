%% params_inversor.m
% Inicialización de los parámetros, matrices, límites de saturación y
% condiciones iniciales del inversor trifásico no ideal.
%
% El inversor se modela mediante tres canales idénticos, uno por fase:
%
%   v_f^*(t) --> filtro pasa-bajos de 2.º orden --> saturación --> v_f(t)
%
% donde f pertenece a {a,b,c}. El filtro tiene ganancia estática unitaria,
% frecuencia natural wn_v y relación de amortiguamiento zeta_v.
%
% Variables principales para Simulink:
%
%   Filtro por fase:      A_v, B_v, C_v, D_v
%   Saturación:           V_sat_max, V_sat_min
%   Condiciones iniciales:
%                        X0_va, X0_vb, X0_vc
%
% También se conserva X0_v como alias de la condición inicial común usada
% en el desarrollo original.
%
% Ejecutar este archivo antes de iniciar la simulación.
% No se utiliza "clear" para no eliminar otros parámetros del workspace.

%% 1. Valores límite de tensión del inversor
% La especificación proporciona la tensión máxima de línea, expresada como
% valor eficaz:
%
%   V_sl_max = 48 Vca rms
%
% Para un sistema trifásico equilibrado conectado en estrella:
%
%   V_sf_max_rms  = V_sl_max/sqrt(3)
%   V_sf_max_pico = sqrt(2)*V_sf_max_rms
%
% Por lo tanto, cada tensión instantánea de fase debe cumplir:
%
%   |v_as(t)|, |v_bs(t)|, |v_cs(t)| <= V_sf_max_pico

V_sl_max = 48;                         % Tensión máxima de línea [V rms]
V_sf_max_rms = V_sl_max/sqrt(3);       % Tensión máxima fase-neutro [V rms]
V_sf_max_pico = sqrt(2)*V_sf_max_rms;  % Tensión máxima de fase [V pico]

% Variables para los bloques Saturation de Simulink.
V_sat_max = V_sf_max_pico;             % Límite superior [V]
V_sat_min = -V_sf_max_pico;            % Límite inferior [V]

% Alias conservado por compatibilidad con el desarrollo original.
V_fase_max = V_sf_max_pico;

% Valores numéricos esperados:
% V_sf_max_rms  = 27.7128 V rms
% V_sf_max_pico = 39.1918 V pico
% Saturación por fase: -39.1918 V <= v_f(t) <= 39.1918 V

%% 2. Modelo dinámico del inversor: filtro pasa-bajos de segundo orden
% Para cada una de las tres fases:
%
%                        wn_v^2
% G_v(s) = -------------------------------------
%          s^2 + 2*zeta_v*wn_v*s + wn_v^2
%
% El modelo posee ganancia estática unitaria. El valor especificado para el
% inversor no ideal es wn_v = 6000 rad/s y zeta_v = 1.

k_v = 1;               % Ganancia estática nominal del inversor
k_w_v = 4.5;             % Multiplicador para análisis de sensibilidad
wn_v_base = 6000;      % Frecuencia natural especificada [rad/s]
zeta_v = 1;            % Amortiguamiento crítico [-]

wn_v = k_w_v*wn_v_base;  % Frecuencia natural efectiva [rad/s]
f_n_v = wn_v/(2*pi);     % Frecuencia natural equivalente [Hz]

% Coeficientes de la función de transferencia.
num_v = k_v*wn_v^2;
den_v = [1, 2*zeta_v*wn_v, wn_v^2];

% Realización en espacio de estados utilizada en el modelo:
%
%   x_dot = A_v*x + B_v*v_f^*
%   v_f   = C_v*x + D_v*v_f^*

A_v = [0,       -1;
       wn_v^2,  -2*zeta_v*wn_v];

B_v = [1;
       0];

C_v = [0, k_v];
D_v = 0;

% Con k_w_v = 1, k_v = 1 y zeta_v = 1:
% A_v = [0,       -1;
%        3.6e7,   -1.2e4]
%
% B_v = [1; 0]
% C_v = [0, 1]
% D_v = 0
%
% Los dos polos coinciden en s = -6000 rad/s.

polos_v = eig(A_v);
tau_polo_v = 1/wn_v;  % Constante de tiempo asociada a cada polo [s]

% Ganancia estática calculada directamente desde el modelo en SS.
K_dc_v = D_v - C_v*(A_v\B_v);

%% 3. Condiciones iniciales de los estados del inversor
% Se calculan en estado estacionario para evitar un transitorio artificial
% al comenzar la simulación:
%
%   0  = A_v*X0 + B_v*v_f0
%   v_f0 = C_v*X0 + D_v*v_f0
%
% Para la realización utilizada:
%
%   X0_vf = [(2*zeta_v/wn_v)*v_f0;
%            v_f0]
%
% En estado estacionario, la salida inicial del filtro es k_v*v_f0.
% Para el inversor especificado k_v = 1, por lo que la tensión aplicada
% comienza con el mismo valor que la tensión comandada.
%
% Los valores siguientes representan las tensiones comandadas al filtro en
% t = 0. Deben modificarse únicamente si la simulación comienza desde un
% punto de operación con tensiones de fase no nulas.

v_a0 = 0;  % Tensión inicial comandada de fase a [V]
v_b0 = 0;  % Tensión inicial comandada de fase b [V]
v_c0 = 0;  % Tensión inicial comandada de fase c [V]

v_abc0 = [v_a0;
          v_b0;
          v_c0];

X0_va = [(2*zeta_v/wn_v)*v_a0;
         v_a0];

X0_vb = [(2*zeta_v/wn_v)*v_b0;
         v_b0];

X0_vc = [(2*zeta_v/wn_v)*v_c0;
         v_c0];

% Matriz auxiliar: cada columna contiene el estado inicial de una fase.
X0_vabc = [X0_va, X0_vb, X0_vc];

% Alias para modelos que utilizan la misma condición inicial en las tres
% fases. Con el arranque nominal, X0_v = [0; 0].
X0_v = X0_va;

%% 4. Comprobaciones básicas
assert(V_sl_max > 0,  'V_sl_max debe ser positivo.');
assert(k_v > 0,       'k_v debe ser positivo.');
assert(k_w_v > 0,     'k_w_v debe ser positivo.');
assert(wn_v_base > 0, 'wn_v_base debe ser positivo.');
assert(wn_v > 0,      'wn_v debe ser positivo.');
assert(zeta_v > 0,    'zeta_v debe ser positivo.');
assert(V_sat_max > 0, 'V_sat_max debe ser positivo.');
assert(V_sat_min < 0, 'V_sat_min debe ser negativo.');

% El filtro debe ser asintóticamente estable.
assert(all(real(polos_v) < 0), ...
       'El modelo dinámico del inversor no es estable.');

% La ganancia estática debe ser unitaria.
tol_inv = 1e-10;
assert(abs(K_dc_v - k_v) < tol_inv, ...
       'La ganancia estática del modelo del inversor no coincide con k_v.');

% Verifica que cada condición inicial reproduzca la tensión inicial elegida.
assert(abs(C_v*X0_va + D_v*v_a0 - k_v*v_a0) < tol_inv, ...
       'La condición inicial X0_va no reproduce la salida inicial esperada.');
assert(abs(C_v*X0_vb + D_v*v_b0 - k_v*v_b0) < tol_inv, ...
       'La condición inicial X0_vb no reproduce la salida inicial esperada.');
assert(abs(C_v*X0_vc + D_v*v_c0 - k_v*v_c0) < tol_inv, ...
       'La condición inicial X0_vc no reproduce la salida inicial esperada.');

% Verifica que los estados iniciales sean estados de equilibrio para las
% entradas iniciales configuradas.
assert(norm(A_v*X0_va + B_v*v_a0, inf) < tol_inv, ...
       'X0_va no es un estado de equilibrio para v_a0.');
assert(norm(A_v*X0_vb + B_v*v_b0, inf) < tol_inv, ...
       'X0_vb no es un estado de equilibrio para v_b0.');
assert(norm(A_v*X0_vc + B_v*v_c0, inf) < tol_inv, ...
       'X0_vc no es un estado de equilibrio para v_c0.');
