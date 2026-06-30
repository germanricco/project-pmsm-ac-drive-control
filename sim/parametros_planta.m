% NOTA IMPORTANTE. PARAMETROS MECANICOS, ELECTRICOS Y TERMICOS SEPARADOS POR MODULARIDAD 
% FALTAN SEPARAR LAS CONDICIONES INICIALES. LAS MATRICES LTI Y FUNCIONES AUXILIARES NO SE QUE ONDA
% 
% Contiene parámetros mecánicos, eléctricos, térmicos, condiciones iniciales,
% matrices LTI nominales y funciones auxiliares para linealización LPV.

clear; clc;

%% =========================================================
%  1. PARÁMETROS MECÁNICOS
% ==========================================================

% Gravedad
g = 9.80665;                       % [m/s^2]

% Reductor planetario
r = 120.0;                         % [-] relación de reducción total

% Motor + caja, referido al eje del motor
Jm = 14.0e-6;                      % [kg*m^2]
bm = 15.0e-6;                      % [N*m/(rad/s)]

% Carga mecánica: péndulo rígido actuado
m = 1.0;                           % [kg] masa del brazo
Lcm = 0.25;                        % [m] distancia al centro de masa
Jcm = 0.0208;                      % [kg*m^2] inercia respecto al CM
Ll = 0.50;                         % [m] longitud total del brazo

% Carga útil
ml_nom = 0.0;                      % [kg] carga útil nominal inicial
ml_min = 0.0;                      % [kg]
ml_max = 1.5;                      % [kg]
ml = ml_nom;                       % [kg] valor usado para el modelo nominal

% Fricción de carga
bl_nom = 0.1;                      % [N*m/(rad/s)]
bl_min = 0.1 - 0.03;               % [N*m/(rad/s)]
bl_max = 0.1 + 0.03;               % [N*m/(rad/s)]
bl = bl_nom;                       % [N*m/(rad/s)] valor usado para el modelo nominal

% Perturbación externa por contacto aplicada en la carga
Tld_max = 5.0;                     % [N*m]
Tld_sim_max = 6.28;                % [N*m] valor pedido para simulación del punto 5.1.6

% Parámetros de carga nominal
Jl = (m*Lcm^2 + Jcm) + ml*Ll^2;    % [kg*m^2]
kl = m*Lcm + ml*Ll;                % [kg*m]

% Rangos por variación de carga útil
Jl_min = (m*Lcm^2 + Jcm) + ml_min*Ll^2;
Jl_max = (m*Lcm^2 + Jcm) + ml_max*Ll^2;
kl_min = m*Lcm + ml_min*Ll;
kl_max = m*Lcm + ml_max*Ll;

% Parámetros equivalentes referidos al eje del motor
Jeq = Jm + Jl/r^2;                 % [kg*m^2]
beq = bm + bl/r^2;                 % [N*m/(rad/s)]
kg_eq = g*kl/r;                   % [N*m] coef. gravitacional referido al motor
Tld_eq_max = Tld_max/r;            % [N*m]
Tld_sim_eq_max = Tld_sim_max/r;    % [N*m]

% Rangos equivalentes
Jeq_min = Jm + Jl_min/r^2;
Jeq_max = Jm + Jl_max/r^2;
beq_min = bm + bl_min/r^2;
beq_nom = bm + bl_nom/r^2;
beq_max = bm + bl_max/r^2;

%% =========================================================
%  2. PARÁMETROS ELÉCTRICOS PMSM
% ==========================================================

Pp = 3;                            % [-] pares de polos
lambda_m = 0.016;                  % [Wb] flujo de imanes concatenado
Lq = 5.8e-3;                       % [H] inductancia eje q
Ld = 6.6e-3;                       % [H] inductancia eje d
Lls = 0.8e-3;                      % [H] inductancia de dispersión

Rs_REF = 1.02;                     % [ohm] resistencia por fase a Ts_REF
Ts_REF = 20.0;                     % [°C]
alpha_Cu = 3.9e-3;                 % [1/°C]

% Función de resistencia de estator dependiente de temperatura
Rs_Ts = @(Ts) Rs_REF*(1 + alpha_Cu*(Ts - Ts_REF));
dRs_dTs = Rs_REF*alpha_Cu;         % [ohm/°C]

% Constantes de torque
Kt = (3/2)*Pp*lambda_m;            % [N*m/A]
Krel = (3/2)*Pp*(Ld - Lq);         % [N*m/A^2]

%% =========================================================
%  3. PARÁMETROS TÉRMICOS
% ==========================================================

Cts = 0.818;                       % [W/(°C/s)] capacitancia térmica
Rts_amb = 146.7;                   % [°C/W] resistencia térmica estator-ambiente
tau_ts_amb = Rts_amb*Cts;          % [s] constante de tiempo térmica

Tamb_min = -15.0;                  % [°C]
Tamb_max = 40.0;                   % [°C]
Tamb = Tamb_max;                   % [°C] valor inicial recomendado
Ts_max = 115.0;                    % [°C]

%% =========================================================
%  4. ESPECIFICACIONES DE OPERACIÓN
% ==========================================================

% Reductor/carga
nl_nom_rpm = 60.0;                 % [rpm] velocidad nominal salida
wl_nom = 6.28;                     % [rad/s]
Tq_nom = 17.0;                     % [N*m]
Tq_max = 45.0;                     % [N*m]

% Motor
nm_nom_rpm = 6600.0;               % [rpm]
wm_nom = 691.15;                   % [rad/s]
Vsl_nom_rms = 30.0;                % [V rms] tensión nominal de línea
Vsf_nom_rms = Vsl_nom_rms/sqrt(3); % [V rms] tensión nominal de fase
Is_nom_rms = 0.4;                  % [A rms]
Is_max_rms = 2.0;                  % [A rms]

% Inversor idealizado
Vsl_max_rms = 48.0;                % [V rms]
vas_max = sqrt(2)*Vsl_max_rms/sqrt(3);  % [V pico fase] saturación por fase
fe_max = 330.0;                    % [Hz]
we_max = 2*pi*fe_max;              % [rad/s]

% Valor de consigna de tensión q pedido en la guía para simulación del punto 5.1.6
Vqs_nom = 19.596;                  % [V]
Vds_test = Vqs_nom/10;             % [V]

%% =========================================================
%  5. CONDICIONES INICIALES RECOMENDADAS
% ==========================================================

theta_m_0 = 0.0;                   % [rad]
omega_m_0 = 0.0;                   % [rad/s]
iq_0 = 0.0;                        % [A]
id_0 = 0.0;                        % [A]
Ts_0 = Tamb;                       % [°C]

x0_nl = [theta_m_0; omega_m_0; iq_0; id_0; Ts_0];

% Casos pedidos para comparar dinámica residual del eje d
x0_id_pos = [theta_m_0; omega_m_0; iq_0; +0.5; Ts_0];
x0_id_neg = [theta_m_0; omega_m_0; iq_0; -0.5; Ts_0];

%% =========================================================
%  6. MODELO LTI EQUIVALENTE NOMINAL
% ==========================================================
% Modelo con campo orientado id = 0 y desacoplamiento en ejes d-q.
% Estados: x_lti = [theta_m; omega_m; iq; id; Ts]^T
% Entradas: u_lti = [u_q; Tld; P_perd; Tamb]^T
%
% Nota: para el modelo LTI nominal se toma Rs = Rs_REF.
% Si se desea evaluar a 40 °C, reemplazar Rs_nom_lti por Rs_Ts(40).

Rs_nom_lti = Rs_REF;               % [ohm]

% Coeficientes útiles
a_grav = g*kl/(Jeq*r^2);
a_fric = beq/Jeq;
a_iq = Rs_nom_lti/Lq;
a_id = Rs_nom_lti/Ld;
a_th = 1/(Rts_amb*Cts);

A_lti = [ 0        1         0           0       0;
         -a_grav  -a_fric    Kt/Jeq      0       0;
          0        0        -a_iq        0       0;
          0        0         0          -a_id    0;
          0        0         0           0      -a_th ];

B_lti = [ 0        0             0        0;
          0       -1/(Jeq*r)     0        0;
          1/Lq     0             0        0;
          0        0             0        0;
          0        0             1/Cts    1/(Rts_amb*Cts) ];

C_q = [1/r 0 0 0 0];               % salida q = theta_m/r
C_theta_m = [1 0 0 0 0];           % salida theta_m
C_omega_m = [0 1 0 0 0];           % salida omega_m
D_lti = zeros(1,4);

%% =========================================================
%  7. FUNCIONES DE TRANSFERENCIA DEL MODELO LTI
% ==========================================================
% Requiere Control System Toolbox.
% Si no se dispone de esta herramienta, igualmente se dejan definidos
% numeradores y denominadores.

num_theta_uq = (Kt/Jeq)*(1/Lq);
den_theta_uq = conv([1 Rs_nom_lti/Lq], [1 beq/Jeq g*kl/(Jeq*r^2)]);

num_theta_Tld = -1/(Jeq*r);
den_theta_Tld = [1 beq/Jeq g*kl/(Jeq*r^2)];

if exist('tf','file') == 2
    G_theta_uq = tf(num_theta_uq, den_theta_uq);
    G_theta_Tld = tf(num_theta_Tld, den_theta_Tld);
else
    G_theta_uq = [];
    G_theta_Tld = [];
end

%% =========================================================
%  8. FUNCIÓN PARA MATRICES LPV JACOBIANAS
% ==========================================================
% Punto de operación:
%   theta0 : [rad]
%   wm0    : [rad/s]
%   Iq0    : [A]
%   Id0    : [A]
%   Ts0    : [°C]
%
% Entradas del modelo LPV:
%   Delta u = [Delta vq; Delta vd; Delta Tld; Delta Tamb]

A_lpv_fun = @(theta0, wm0, Iq0, Id0, Ts0) [ ...
    0, 1, 0, 0, 0; ...
    -g*kl/(Jeq*r^2)*cos(theta0/r), -beq/Jeq, (Kt + Krel*Id0)/Jeq, Krel*Iq0/Jeq, 0; ...
    0, -Pp*(lambda_m + Ld*Id0)/Lq, -Rs_Ts(Ts0)/Lq, -Ld*Pp*wm0/Lq, -dRs_dTs*Iq0/Lq; ...
    0, Lq*Pp*Iq0/Ld, Lq*Pp*wm0/Ld, -Rs_Ts(Ts0)/Ld, -dRs_dTs*Id0/Ld; ...
    0, 0, 3*Rs_Ts(Ts0)*Iq0/Cts, 3*Rs_Ts(Ts0)*Id0/Cts, ((3/2)*dRs_dTs*(Iq0^2 + Id0^2) - 1/Rts_amb)/Cts ...
];

B_lpv = [ 0,      0,        0,              0;
          0,      0,       -1/(Jeq*r),      0;
          1/Lq,   0,        0,              0;
          0,      1/Ld,     0,              0;
          0,      0,        0,              1/(Rts_amb*Cts) ];

%% =========================================================
%  9. LEYES DE DESACOPLAMIENTO PARA SIMULINK
% ==========================================================
% Estas expresiones se implementan como bloques o MATLAB Function.
%
% omega_r = Pp*omega_m
% vd_star = -Lq*iq*omega_r
% vq_star = uq + (lambda_m + Ld*id)*omega_r
%
% En Simulink:
%   entrada auxiliar: uq
%   entrada real al modelo PMSM: vq = uq + (lambda_m + Ld*id)*Pp*omega_m
%   entrada real al eje d: vd = -Lq*iq*Pp*omega_m

%% =========================================================
%  10. MOSTRAR RESULTADOS PRINCIPALES
% ==========================================================

fprintf('\n================ PARAMETROS MECANICOS ================\n');
fprintf('Jl                  = %.8e kg*m^2\n', Jl);
fprintf('kl                  = %.8e kg*m\n', kl);
fprintf('Jeq                 = %.8e kg*m^2\n', Jeq);
fprintf('beq                 = %.8e N*m/(rad/s)\n', beq);
fprintf('g*kl/r              = %.8e N*m\n', kg_eq);
fprintf('Tld_eq_max          = %.8e N*m\n', Tld_eq_max);

fprintf('\n================ PARAMETROS ELECTRICOS ================\n');
fprintf('Kt                  = %.8e N*m/A\n', Kt);
fprintf('Krel                = %.8e N*m/A^2\n', Krel);
fprintf('Rs_REF              = %.8e ohm\n', Rs_REF);
fprintf('Rs(Ts=40 C)         = %.8e ohm\n', Rs_Ts(40));
fprintf('Rs/Lq               = %.8e rad/s\n', Rs_nom_lti/Lq);
fprintf('Rs/Ld               = %.8e rad/s\n', Rs_nom_lti/Ld);

fprintf('\n================ PARAMETROS TERMICOS ===================\n');
fprintf('tau_ts_amb          = %.8e s\n', tau_ts_amb);
fprintf('1/(Rts_amb*Cts)     = %.8e 1/s\n', a_th);

fprintf('\n================ MATRIZ A_LTI ==========================\n');
disp(A_lti);

fprintf('\n================ MATRIZ B_LTI ==========================\n');
disp(B_lti);

fprintf('\n================ FT theta_m/u_q ========================\n');
fprintf('Numerador           = %.8e\n', num_theta_uq);
fprintf('Denominador         = [%.8e %.8e %.8e %.8e]\n', den_theta_uq);

fprintf('\n================ FT theta_m/Tld ========================\n');
fprintf('Numerador           = %.8e\n', num_theta_Tld);
fprintf('Denominador         = [%.8e %.8e %.8e]\n', den_theta_Tld);

