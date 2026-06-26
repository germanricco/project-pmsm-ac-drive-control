% =========================================================
%  2. PARÁMETROS ELÉCTRICOS PMSM
% =========================================================
Pp = 3;                            % [-] Pares de polos
lambda_m = 0.016;                  % [Wb-turn] [V/(rad/s)] Flujo de imanes concatenado
L_q = 5.8e-3;                      % [H] Inductancia eje q
L_d = 6.6e-3;                      % [H] Inductancia eje d
L_l = 0.8e-3;                     % [H] Inductancia de dispersión

Rs_REF = 1.02;                     % [ohm] Resistencia por fase a Ts_REF
Ts_REF = 20.0;                     % [°C] Temperatura de estator de referencia

% Constantes de torque
K_t = (3/2)*Pp*lambda_m;           % [N*m/A]
K_rel = (3/2)*Pp*(L_d - L_q);      % [N*m/A^2]

alpha_Cu = 3.9e-3;                 % [1/°C] Coeficiente de Aumento de Rs con Ts