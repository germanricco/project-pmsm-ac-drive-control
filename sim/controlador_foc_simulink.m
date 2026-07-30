function [v_a, v_b, v_c] = controlador_foc_simulink(i_a, i_b, i_c, theta_m, T_s_segura, q_t, q_prime_t)


    %% SECCIÓN 1: CONFIGURACION Y VAR. GLOBALES (#define y variables globales)
    % En la generación de código C, el compilador tratará todo esto como constantes.

    % ===== PARAMETRO DE MUESTREO =====
    T_s = 100e-6;   % [s] 10 KHz
    
    % ===== SUBSISTEMA MECANICO =====
    g = 9.80665;    % [m/s^2] Aceleración de la gravedad
    r = 120.0;      % [-] Relación de reducción total
    
    bl_nom = 0.10;          % [N*m/(rad/s)]
    m   = 1.0;       % [kg] Masa del brazo
    Lcm = 0.25;      % [m] Distancia del centro de masa a la articulación
    Jcm = 0.0208;    % [kg*m^2] Inercia respecto del centro de masa
    Ll  = 0.50;      % [m] Longitud hasta el extremo
    ml_nom = 0.0;    % [kg]

    Jm = 14.0e-6;    % [kg*m^2]
    bm = 15.0e-6;    % [N*m/(rad/s)]

    Jl_nom = (m*Lcm^2 + Jcm) + ml_nom*Ll^2;
    kl_nom = m*Lcm + ml_nom*Ll;
    
    Jeq_ctrl   = Jm + Jl_nom/r^2;
    beq_ctrl   = bm + bl_nom/r^2;
    kg_eq_ctrl = g*kl_nom/r;
    Jeq_obs    = Jeq_ctrl;
    beq_obs    = beq_ctrl;
    
    % ===== SUBSISTEMA ELECTRICO =====
    Pp       = 3;          % [-] Pares de polos
    lambda_m = 0.016;      % [Wb] Flujo concatenado de los imanes
    L_q      = 5.8e-3;     % [H] Inductancia del eje q
    L_d      = 6.6e-3;     % [H] Inductancia del eje d
    L_l      = 0.8e-3;     % [H] Inductancia de dispersión / eje cero
    
    K_t   = (3/2)*Pp*lambda_m;       % [N*m/A] Constante de torque con i_d = 0
    K_rel = (3/2)*Pp*(L_d - L_q);    % [N*m/A^2] Componente de reluctancia
    
    % ===== SUBSISTEMA TÉRMICO =====
    Rs_REF   = 1.02;     % [ohm/fase] A Ts_REF
    Ts_REF   = 20.0;     % [°C]
    alpha_Cu = 3.9e-3;   % [1/°C]
    
    % ===== GANANCIAS OBSERVADOR DE ESTADO =====
    polo_obs_deseado = -3200;    % [rad/s] Dos polos coincidentes
    K_eth_num = 3 * abs(polo_obs_deseado);
    K_ew_num  = 3 * abs(polo_obs_deseado)^2;
    K_ei_num  = abs(polo_obs_deseado)^3;
    
    % ===== GANANCIAS CONTROLADORES =====
    p_c = -5000;                 % [rad/s] Polo deseado de los lazos de corriente
    R_q_num = -p_c*L_q;          % [ohm]
    R_d_num = -p_c*L_d;          % [ohm]
    R_o_num = -p_c*L_l;          % [ohm]
    
    wn_pid   = 800;              % [rad/s]
    zeta_pid = 0.75;             % [-]
    n_pid    = 2*zeta_pid + 1;   % n = 2.5
    
    ba_num   = Jeq_ctrl*n_pid*wn_pid;
    Ksa_num  = Jeq_ctrl*n_pid*wn_pid^2;
    Ksia_num = Jeq_ctrl*wn_pid^3;
    Torque_max = 45;

    %% SECCIÓN 2: INICIALIZACIÓN DE MEMORIA (ESTADOS)
    persistent T_m_star_prev
    
    if isempty(T_m_star_prev)
        T_m_star_prev = 0;
    end
    
    % Consignas fijas para control FOC estándar
    i_d_star = 0; 
    i_o_star = 0;
    
    %% SECCIÓN 3: EJECUCIÓN DEL LAZO (Simula 1 ciclo del Timer)
    % Se ejecutara una vez cada T_s segundos

    % 1. Preparacion 
    theta_r = calcular_theta_r(theta_m);
    R_s = estimar_Rs(T_s_segura);
        
    % Transformacion de Park (abc -> qd0)
    [i_q, i_d, i_o] = T_Park(i_a, i_b, i_c, theta_r);
        
    % Observador de Estado (Usando el T_m_star del ciclo anterior)
    [w_m_estimada, theta_m_estimada] = observador_estado(theta_m, T_m_star_prev);
        
    % Lazo Externo (Velocidad / Posición)
    [theta_m_star, w_m_star] = set_point_discreto(q_t, q_prime_t);

    T_m_star = controlador_pid(w_m_star, w_m_estimada);
        
    % Lazo Interno (Corriente)
    i_q_star = compensador_mecanico(T_m_star, w_m_estimada, theta_m, i_d);

    [v_q, v_d, v_o] = controlador_corriente(i_q_star, i_d_star, i_o_star, i_q, i_d, i_o, R_s, w_m_estimada);
                                                
    % Transformacion Inversa de Park (qd0 -> abc)
    [v_a, v_b, v_c] = TI_Park(v_q, v_d, v_o, theta_r);
        
    % Actualizar la Memoria para proximo ciclo
    T_m_star_prev = T_m_star; 
    
    %% SECCIÓN 4: MODULOS DE CONTROL
    % Funciones que toman mediciones o calculan referencias
    
    function [theta_m_star, w_m_star] = set_point_discreto(q_t, q_prime_t)
        % Definición de variables de memoria (persistent)
        persistent q_t_prev q_prime_prev theta_int_prev
    
        % Inicialización en el primer ciclo de ejecución (t=0)
        if isempty(q_t_prev)
            q_t_prev = q_t;             % Evita el pico (infinito) de la derivada en t=0
            q_prime_prev = q_prime_t;
            theta_int_prev = 0;         % El integrador arranca descargado
        end
    
        % --- CÁLCULO NUMÉRICO DISCRETO ---
        % A. Derivador Discreto (Diferencia Hacia Atrás / Backward Euler)
        q_dot_derivado = (q_t - q_t_prev) / T_s;
    
        % B. Integrador Discreto (Método de Tustin / Trapezoidal)
        theta_integrado = theta_int_prev + (T_s / 2) * (q_prime_t + q_prime_prev);
    
        % --- SALIDAS DEL SISTEMA ---
        theta_m_star = r * (q_t + theta_integrado);
        w_m_star     = r * (q_prime_t + q_dot_derivado);
    
        % Actualización de memoria para el próximo ciclo
        q_t_prev = q_t;
        q_prime_prev = q_prime_t;
        theta_int_prev = theta_integrado;
    end
    
    function T_m_star = controlador_pid(w_m_star, w_m)
        % Memoria de estados para los integradores y valores previos
        persistent e_pos_estado e_int_estado e_vel_prev e_pos_prev
        
        % Inicialización en el primer ciclo (t=0)
        if isempty(e_pos_estado)
            e_pos_estado = 0; % Memoria del integrador de posición
            e_int_estado = 0; % Memoria del integrador de la acción integral
            e_vel_prev = 0;   % Velocidad del ciclo anterior
            e_pos_prev = 0;   % Posición del ciclo anterior
        end
    
        % Cálculo del Error de Velocidad
        e_vel = w_m_star - w_m;
    
        % Primer Integrador: Error de Velocidad -> Error de Posición
        e_pos = e_pos_estado + (T_s / 2) * (e_vel + e_vel_prev);
        
        % Segundo Integrador: Error de Posición -> Integral del error de posición
        e_int = e_int_estado + (T_s / 2) * (e_pos + e_pos_prev);
        
        D_action = ba_num * e_vel;
        P_action = Ksa_num * e_pos;
        I_action = Ksia_num * e_int;
    
        % --- CALCULO DE LA SALIDA ---
        T_m_star_unbounded = P_action + I_action + D_action;
    
        % --- SATURACIÓN DE SALIDA ---
        if T_m_star_unbounded > Torque_max
            T_m_star = Torque_max;
            % Saturado: No guardamos el nuevo e_int en e_int_estado (congelamos el integrador I)
        elseif T_m_star_unbounded < -Torque_max
            T_m_star = -Torque_max;
            % Saturado: No guardamos el nuevo e_int en e_int_estado (congelamos el integrador I)
        else
            T_m_star = T_m_star_unbounded;
            % Zona lineal: Actualizamos la memoria del integrador I
            e_int_estado = e_int; 
        end
    
        % El integrador de posición NUNCA se congela, solo el de integracion.
        e_pos_estado = e_pos;
    
        % Actualización de memorias para el cálculo de Tustin en el próximo ciclo
        e_vel_prev = e_vel;
        e_pos_prev = e_pos;
    end
    
    function i_q_star = compensador_mecanico(T_m_star, w_m_estimada, theta_m, i_d)
        % Compensación Feedforward de Fricción Viscosa
        T_friccion = beq_ctrl * w_m_estimada;
    
        % Compensación Feedforward de Gravedad
        T_gravedad = kg_eq_ctrl * sin(theta_m / r);
    
        % Torque Total Requerido
        T_req = T_m_star + T_friccion + T_gravedad;
    
        % --- Modulación de Torque a Corriente ---
        % T = (K_t + K_rel * i_d) * i_q
        Denominador_Torque = K_t + K_rel * i_d;
    
        % Evitar división por cero
        if abs(Denominador_Torque) < 1e-6
            Denominador_Torque = sign(Denominador_Torque) * 1e-6;
            if Denominador_Torque == 0
                Denominador_Torque = 1e-6;
            end
        end
    
        % Cálculo final de la consigna de corriente
        i_q_star = T_req / Denominador_Torque;
    end
    
    function [v_q, v_d, v_o] = controlador_corriente(i_q_star, i_d_star, i_o_star, i_q, i_d, i_o, R_s, w_m_estimada)
        % --- Controlador Proporcional ---
        % Cálculo del error de seguimiento
        e_q = i_q_star - i_q;
        e_d = i_d_star - i_d;
        e_o = i_o_star - i_o;
        
        % Generación de las tensiones de control
        v_bar_q = R_q_num * e_q;
        v_bar_d = R_d_num * e_d;
        v_bar_o = R_o_num * e_o;
    
        % --- Desacoplamiento electrico (Feedback Linearization) ---
        % Eje en Cuadratura (q): Compensa caída óhmica y fuerza contraelectromotriz
        v_q = v_bar_q + R_s * i_q + Pp * w_m_estimada * (L_d * i_d + lambda_m);
        
        % Eje Directo (d): Compensa caída óhmica y acoplamiento por reluctancia cruzada
        v_d = v_bar_d + R_s * i_d - Pp * w_m_estimada * L_q * i_q;
        
        % Eje Homopolar (0): Compensa únicamente caída óhmica
        v_o = v_bar_o + R_s * i_o;
    end
    
    %% SECCIÓN 5: OBSERVADORES Y ESTIMADORES
    % Funciones que infieren el estado del sistema a partir de mediciones
    
    function [w_m_estimada, theta_m_estimada] = observador_estado(theta_m, T_m_star)
        % Memoria de los integradores (Salidas)
        persistent int1_estado w_est_estado th_est_estado
        
        % Memoria de las entradas a los integradores (para método de Tustin)
        persistent in1_prev in2_prev in3_prev
        
        % Inicialización
        if isempty(th_est_estado)
            int1_estado = 0;   
            w_est_estado = 0;    
            th_est_estado = 0;  
            
            in1_prev = 0;
            in2_prev = 0;
            in3_prev = 0;
        end
        
        % Cálculo del error de estimación (usando la posición estimada previa)
        e_theta = theta_m - th_est_estado;
        
        % --- Lazo de Perturbación (K_ei) ---
        in1_actual = K_ei_num * e_theta;
        int1_actual = int1_estado + (T_s / 2) * (in1_actual + in1_prev); % Salida del 1er integrador
        
        % --- Lazo de Velocidad (K_ew) ---
        in2_actual = (T_m_star / Jeq_obs) + int1_actual + (K_ew_num * e_theta);
        w_est_actual = w_est_estado + (T_s / 2) * (in2_actual + in2_prev); % Salida del 2do integrador
        
        % --- Lazo de Posición (K_eth) ---
        in3_actual = w_est_actual + (K_eth_num * e_theta);
        th_est_actual = th_est_estado + (T_s / 2) * (in3_actual + in3_prev); % Salida del 3er integrador
        
        % --- ASIGNACIÓN DE SALIDAS ---
        w_m_estimada = w_est_actual;
        theta_m_estimada = th_est_actual;
        
        % --- ACTUALIZACIÓN DE MEMORIAS PARA EL PRÓXIMO CICLO ---
        % Actualizamos los estados de los integradores
        int1_estado = int1_actual;
        w_est_estado = w_est_actual;
        th_est_estado = th_est_actual;
        
        % Actualizamos las entradas previas para el cálculo de Tustin
        in1_prev = in1_actual;
        in2_prev = in2_actual;
        in3_prev = in3_actual;
    end
    
    function R_s = estimar_Rs(T_s_segura)
        % Cálculo del incremento de temperatura
        Delta_T = T_s_segura - Ts_REF;
    
        % Aplicación de la Ecuación del modelo físico
        R_s = Rs_REF * (1 + alpha_Cu * Delta_T);
    end
    
    %% SECCIÓN 6: MATEMÁTICA Y TRANSFORAMACIONES (utils.c)
    % Funciones algebraicas, sin memoria ni estados
    
    function theta_r = calcular_theta_r(theta_m)
        theta_r = Pp * theta_m;
    end
    
    function [fq, fd, f0] = T_Park(fa, fb, fc, theta_r)
        % Transformación directa de Park: abc -> qd0
        c1 = cos(theta_r);
        c2 = cos(theta_r - 2*pi/3);
        c3 = cos(theta_r + 2*pi/3);
        
        s1 = sin(theta_r);
        s2 = sin(theta_r - 2*pi/3);
        s3 = sin(theta_r + 2*pi/3);
        
        fq = (2/3)*(c1*fa + c2*fb + c3*fc);
        fd = (2/3)*(s1*fa + s2*fb + s3*fc);
        f0 = (1/3)*(fa + fb + fc);
    end
    
    function [fa, fb, fc] = TI_Park(fq, fd, f0, theta_r)
        % Transformación inversa de Park: qd0 -> abc
        fa = cos(theta_r)*fq ...
           + sin(theta_r)*fd ...
           + f0;
        
        fb = cos(theta_r - 2*pi/3)*fq ...
           + sin(theta_r - 2*pi/3)*fd ...
           + f0;
        
        fc = cos(theta_r + 2*pi/3)*fq ...
           + sin(theta_r + 2*pi/3)*fd ...
           + f0;
    end
end
