function [tabla_limites, tabla_observador, datos, tabla_diagnostico] = ...
    calcular_metricas_operacion(out, planta, t_Tld, Tld, opciones)
%CALCULAR_METRICAS_OPERACION Verifica las especificaciones físicas del proyecto.
%
% La verificación se organiza de acuerdo con la Guía de Trabajo:
%   - Caja reductora: velocidad máxima, torque RMS y torque pico.
%   - PMSM: velocidad, corriente RMS continua, corriente máxima durante
%     aceleraciones, tensión nominal y temperatura.
%   - Inversor: módulo de tensión de línea, saturación instantánea de fase y,
%     cuando planta.Pp está disponible, frecuencia sincrónica estimada.
%
% La corriente máxima de corta duración NO se calcula con una ventana fija
% de 20 ms. Para un sistema trifásico equilibrado se utiliza la envolvente
% RMS equivalente:
%
%   I_fase,rms,eq(t) = sqrt((ia(t)^2 + ib(t)^2 + ic(t)^2)/3)
%
% y se busca su máximo en los intervalos de aceleración. Esta definición no
% depende de que la frecuencia eléctrica sea 50 Hz y se ajusta mejor a la
% especificación "2 A RMS (corta duración, aceleración)".
%
% ENTRADAS
%   out      : Simulink.SimulationOutput con señales en out.yout.
%   planta   : struct con r, Jl, bl, g, kl. Opcionalmente Pp.
%   t_Tld    : tiempos de la perturbación de torque de carga.
%   Tld      : valores de la perturbación de torque de carga.
%   opciones : struct opcional con los campos:
%       intervalo_operacion       [t0 tf]. Por defecto, toda la simulación.
%       intervalos_aceleracion    matriz N x 2 [tinicio tfin]. Si se omite,
%                                 se infieren a partir de w_l_ref.
%       margen_aceleracion        margen temporal alrededor de cada evento
%                                 para incluir la respuesta dinámica.
%                                 Por defecto: 0 s.
%       umbral_aceleracion_rel    umbral relativo para detección automática.
%                                 Por defecto: 0.05.
%
% SALIDAS
%   tabla_limites      : métricas formales frente a valores límite.
%   tabla_observador   : errores máximo y RMS del observador.
%   datos              : señales y métricas derivadas para gráficos.
%   tabla_diagnostico  : métricas complementarias sin criterio formal.
%
% Requiere en out.yout:
%   theta_m, theta_m_ref, theta_m_est
%   w_m, w_m_ref, w_m_est
%   T_m, T_s, i_abc, v_abc_ref
%
% NOTA SOBRE v_abc_ref
% Se interpreta como la tensión solicitada al inversor. Cuando se incorpore
% el modulador no ideal conviene sustituirla por la tensión realmente
% aplicada, si dicha señal se registra con otro nombre.

arguments
    out
    planta struct
    t_Tld (:,1) double
    Tld (:,1) double
    opciones struct = struct()
end

campos = {'r','Jl','bl','g','kl'};
for k = 1:numel(campos)
    assert(isfield(planta,campos{k}), 'Falta planta.%s.', campos{k});
end
assert(numel(t_Tld) == numel(Tld), ...
    't_Tld y Tld deben tener igual longitud.');

%% Límites de la guía vigente
L = struct;
L.wl_max = 6.28;             % rad/s, salida de la caja
L.Tq_rms = 17.0;             % N*m, régimen continuo o RMS
L.Tq_pico = 45.0;            % N*m, corta duración / aceleración
L.wm_max = 691.15;           % rad/s, rotor
L.I_rms_nom = 0.4;           % A RMS, régimen continuo
L.I_rms_max = 2.0;           % A RMS, corta duración / aceleración
L.Vll_nom = 30.0;            % V RMS, motor
L.Vll_max = 48.0;            % V RMS, inversor
L.Vfase_pico_max = sqrt(2)*L.Vll_max/sqrt(3); % V pico por fase
L.Ts_max = 115.0;            % degC
L.fe_max = 330.0;            % Hz

%% Señales principales y estimadas
[theta_ts, theta_ref_ts, theta_est_ts, wm_ts, wm_ref_ts, wm_est_ts, ...
    Tm_ts, Ts_ts, iabc_ts, vabc_ts] = obtener_senales(out);

%% Base temporal mecánica
t = theta_ts.Time(:);
theta_m = vector_columna(theta_ts.Data);
theta_ref = interp_ts(theta_ref_ts,t);
theta_est = interp_ts(theta_est_ts,t);

q     = theta_m/planta.r;
q_ref = theta_ref/planta.r;
q_est = theta_est/planta.r;

e_q         = q_ref-q;
e_theta_obs = theta_m-theta_est;
e_q_obs     = q-q_est;

wm     = interp_ts(wm_ts,t);
wm_ref = interp_ts(wm_ref_ts,t);
wm_est = interp_ts(wm_est_ts,t);

wl     = wm/planta.r;
wl_ref = wm_ref/planta.r;
wl_est = wm_est/planta.r;

e_w_obs  = wm-wm_est;
e_wl_obs = wl-wl_est;

alpha_l = derivada_numerica(t,wl);
alpha_l_ref = derivada_numerica(t,wl_ref);

%% Intervalos de análisis
intervalo_operacion = obtener_opcion(opciones,'intervalo_operacion',[t(1) t(end)]);
validar_intervalo(intervalo_operacion,'intervalo_operacion');
intervalo_operacion(1) = max(intervalo_operacion(1),t(1));
intervalo_operacion(2) = min(intervalo_operacion(2),t(end));

if isfield(opciones,'intervalos_aceleracion') && ...
        ~isempty(opciones.intervalos_aceleracion)
    intervalos_aceleracion = opciones.intervalos_aceleracion;
    validar_intervalos(intervalos_aceleracion,'intervalos_aceleracion');
else
    umbral_rel = obtener_opcion(opciones,'umbral_aceleracion_rel',0.05);
    intervalos_aceleracion = detectar_intervalos_aceleracion( ...
        t,alpha_l_ref,intervalo_operacion,umbral_rel);
end

margen_aceleracion = obtener_opcion(opciones,'margen_aceleracion',0.0);
assert(isscalar(margen_aceleracion) && margen_aceleracion >= 0, ...
    'opciones.margen_aceleracion debe ser un escalar no negativo.');
intervalos_aceleracion = expandir_y_recortar_intervalos( ...
    intervalos_aceleracion,margen_aceleracion,intervalo_operacion);

mask_op_t = mascara_intervalos(t,intervalo_operacion);
assert(nnz(mask_op_t) >= 2, ...
    'El intervalo de operación no contiene suficientes muestras.');

%% Torque mecánico en la salida de la caja
Tld_t = interp1(t_Tld,Tld,t,'previous','extrap');
Tq = planta.Jl.*alpha_l + planta.bl.*wl + ...
     planta.g*planta.kl.*sin(q) + Tld_t;

Tm = interp_ts(Tm_ts,t);
Ts = interp_ts(Ts_ts,t);

torque_rms_op = rms_temporal(t(mask_op_t),Tq(mask_op_t));
torque_pico_op = max(abs(Tq(mask_op_t)));

%% Corrientes y tensiones trifásicas
Iabc = matriz_por_tiempo(iabc_ts);
Vabc = matriz_por_tiempo(vabc_ts);
assert(size(Iabc,2) >= 3, 'i_abc debe contener tres fases.');
assert(size(Vabc,2) >= 3, 'v_abc_ref debe contener tres fases.');
Iabc = Iabc(:,1:3);
Vabc = Vabc(:,1:3);

ti = iabc_ts.Time(:);
tv = vabc_ts.Time(:);
mask_op_i = mascara_intervalos(ti,intervalo_operacion);
mask_op_v = mascara_intervalos(tv,intervalo_operacion);
assert(nnz(mask_op_i) >= 2, 'No hay suficientes muestras de corriente.');
assert(nnz(mask_op_v) >= 2, 'No hay suficientes muestras de tensión.');

% Régimen continuo: RMS temporal por fase durante el ciclo de operación.
irms_fase = rms_temporal(ti(mask_op_i),Iabc(mask_op_i,:));
irms_max = max(irms_fase);

% Envolvente RMS equivalente trifásica. Para un sistema equilibrado y
% senoidal coincide con el RMS de cualquiera de las fases, pero se obtiene
% sin elegir una ventana asociada a una frecuencia fija.
i_rms_equiv = sqrt(sum(Iabc.^2,2)/3);

mask_acel_i = mascara_intervalos(ti,intervalos_aceleracion);
if nnz(mask_acel_i) < 1
    warning(['No se identificaron intervalos de aceleración. ', ...
        'La corriente máxima de corta duración se evaluará en todo el ciclo.']);
    mask_acel_i = mask_op_i;
    ambito_corriente_corta = "Todo el ciclo (sin intervalos de aceleración)";
else
    mask_acel_i = mask_acel_i & mask_op_i;
    ambito_corriente_corta = "Intervalos de aceleración";
end
i_rms_equiv_acel_max = max(i_rms_equiv(mask_acel_i));
i_rms_equiv_ciclo_max = max(i_rms_equiv(mask_op_i));
i_pico = max(abs(Iabc(mask_op_i,:)),[],'all');

% Se conserva únicamente como diagnóstico de compatibilidad con resultados
% anteriores; NO se usa para decidir cumplimiento.
[i_rms_20ms_max, i_rms_20ms_fases] = rms_movil_max(ti,Iabc,20e-3);

Vll = [Vabc(:,1)-Vabc(:,2), ...
       Vabc(:,2)-Vabc(:,3), ...
       Vabc(:,3)-Vabc(:,1)];

% RMS temporal global, solo como diagnóstico.
vll_rms = rms_temporal(tv(mask_op_v),Vll(mask_op_v,:));

% Envolvente RMS equivalente de línea. En un sistema trifásico equilibrado
% equivale al valor RMS de línea instantáneamente asociado a la amplitud.
vll_rms_equiv = sqrt(sum(Vll.^2,2)/3);
vll_rms_equiv_max = max(vll_rms_equiv(mask_op_v));
vfase_pico = max(abs(Vabc(mask_op_v,:)),[],'all');

%% Frecuencia eléctrica estimada desde la velocidad del rotor
if isfield(planta,'Pp') && isscalar(planta.Pp) && isfinite(planta.Pp)
    fe_est = planta.Pp.*wm/(2*pi);
    fe_max = max(abs(fe_est(mask_op_t)));
    comentario_fe = "Derivada de f_e = P_p*w_m/(2*pi)";
else
    fe_est = NaN(size(wm));
    fe_max = NaN;
    comentario_fe = "No evaluada: falta planta.Pp";
end

%% Tabla formal de cumplimiento
Componente = [
    "Caja reductora"
    "Caja reductora"
    "Caja reductora"
    "Motor PMSM"
    "Motor PMSM"
    "Motor PMSM"
    "Motor PMSM"
    "Motor PMSM"
    "Inversor"
    "Inversor"
    "Inversor / PMSM"
    ];

Variable = [
    "Velocidad máxima de carga"
    "Torque de salida RMS del ciclo de operación"
    "Torque de salida pico absoluto"
    "Velocidad máxima del rotor"
    "Corriente de fase RMS del ciclo de operación"
    "Corriente RMS equivalente máxima durante aceleraciones"
    "Tensión línea-línea RMS equivalente máxima"
    "Temperatura máxima del estator"
    "Módulo de tensión de línea RMS equivalente máximo"
    "Tensión de fase instantánea máxima solicitada"
    "Frecuencia sincrónica máxima estimada"
    ];

Criterio = [
    "Máximo absoluto en el ciclo"
    "RMS temporal en el ciclo de operación"
    "Máximo absoluto en el ciclo"
    "Máximo absoluto en el ciclo"
    "Mayor RMS temporal entre las tres fases"
    ambito_corriente_corta
    "Máximo de sqrt((Vab^2+Vbc^2+Vca^2)/3)"
    "Máximo en el ciclo"
    "Máximo de la envolvente RMS de línea"
    "Máximo absoluto entre fases"
    comentario_fe
    ];

Valor = [
    max(abs(wl(mask_op_t)))
    torque_rms_op
    torque_pico_op
    max(abs(wm(mask_op_t)))
    irms_max
    i_rms_equiv_acel_max
    vll_rms_equiv_max
    max(Ts(mask_op_t))
    vll_rms_equiv_max
    vfase_pico
    fe_max
    ];

Limite = [
    L.wl_max
    L.Tq_rms
    L.Tq_pico
    L.wm_max
    L.I_rms_nom
    L.I_rms_max
    L.Vll_nom
    L.Ts_max
    L.Vll_max
    L.Vfase_pico_max
    L.fe_max
    ];

Unidad = [
    "rad/s"
    "N*m RMS"
    "N*m pico"
    "rad/s"
    "A RMS"
    "A RMS equivalente"
    "V línea-línea RMS"
    "degC"
    "V línea-línea RMS"
    "V fase pico"
    "Hz"
    ];

DuracionExceso_s = [
    duracion_condicion(t,abs(wl)>L.wl_max,mask_op_t)
    duracion_condicion(t,abs(Tq)>L.Tq_rms,mask_op_t)
    duracion_condicion(t,abs(Tq)>L.Tq_pico,mask_op_t)
    duracion_condicion(t,abs(wm)>L.wm_max,mask_op_t)
    duracion_condicion(ti,i_rms_equiv>L.I_rms_nom,mask_op_i)
    duracion_condicion(ti,i_rms_equiv>L.I_rms_max,mask_acel_i)
    duracion_condicion(tv,vll_rms_equiv>L.Vll_nom,mask_op_v)
    duracion_condicion(t,Ts>L.Ts_max,mask_op_t)
    duracion_condicion(tv,vll_rms_equiv>L.Vll_max,mask_op_v)
    duracion_condicion(tv,max(abs(Vabc),[],2)>L.Vfase_pico_max,mask_op_v)
    duracion_condicion(t,abs(fe_est)>L.fe_max,mask_op_t)
    ];

[Margen,Uso_pct,Cumple] = evaluar_limites(Valor,Limite);
tabla_limites = table(Componente,Variable,Criterio,Valor,Limite, ...
    Margen,Uso_pct,DuracionExceso_s,Unidad,Cumple);

%% Métricas del observador
VariableObs = [
    "Error máximo de posición estimada (motor)"
    "Error RMS de posición estimada (motor)"
    "Error máximo de velocidad estimada (motor)"
    "Error RMS de velocidad estimada (motor)"
    "Error máximo de posición estimada (carga)"
    "Error RMS de posición estimada (carga)"
    "Error máximo de velocidad estimada (carga)"
    "Error RMS de velocidad estimada (carga)"
    ];

ValorObs = [
    max(abs(e_theta_obs(mask_op_t)))
    rms_temporal(t(mask_op_t),e_theta_obs(mask_op_t))
    max(abs(e_w_obs(mask_op_t)))
    rms_temporal(t(mask_op_t),e_w_obs(mask_op_t))
    max(abs(e_q_obs(mask_op_t)))
    rms_temporal(t(mask_op_t),e_q_obs(mask_op_t))
    max(abs(e_wl_obs(mask_op_t)))
    rms_temporal(t(mask_op_t),e_wl_obs(mask_op_t))
    ];

UnidadObs = [
    "rad"
    "rad RMS"
    "rad/s"
    "rad/s RMS"
    "rad"
    "rad RMS"
    "rad/s"
    "rad/s RMS"
    ];

tabla_observador = table(VariableObs,ValorObs,UnidadObs, ...
    'VariableNames',{'Variable','Valor','Unidad'});

%% Tabla de diagnóstico: informativa, sin criterio directo de la guía
VariableDiag = [
    "Error máximo de seguimiento de posición de carga"
    "Error RMS de seguimiento de posición de carga"
    "Corriente RMS fase a en ciclo de operación"
    "Corriente RMS fase b en ciclo de operación"
    "Corriente RMS fase c en ciclo de operación"
    "Corriente RMS equivalente máxima en todo el ciclo"
    "Corriente instantánea pico"
    "Corriente RMS móvil máxima de 20 ms (solo comparativa)"
    "Tensión RMS temporal Vab en ciclo completo"
    "Tensión RMS temporal Vbc en ciclo completo"
    "Tensión RMS temporal Vca en ciclo completo"
    "Torque electromagnético máximo del motor"
    "Temperatura final del estator"
    ];

ValorDiag = [
    max(abs(e_q(mask_op_t)))
    rms_temporal(t(mask_op_t),e_q(mask_op_t))
    irms_fase(1)
    irms_fase(2)
    irms_fase(3)
    i_rms_equiv_ciclo_max
    i_pico
    i_rms_20ms_max
    vll_rms(1)
    vll_rms(2)
    vll_rms(3)
    max(abs(Tm(mask_op_t)))
    Ts(find(mask_op_t,1,'last'))
    ];

UnidadDiag = [
    "rad"
    "rad RMS"
    "A RMS"
    "A RMS"
    "A RMS"
    "A RMS equivalente"
    "A pico"
    "A RMS (20 ms)"
    "V RMS"
    "V RMS"
    "V RMS"
    "N*m"
    "degC"
    ];

ComentarioDiag = [
    "Métrica de seguimiento, sin límite fijado en la consigna"
    "Métrica de seguimiento, sin límite fijado en la consigna"
    "RMS temporal durante el intervalo de operación"
    "RMS temporal durante el intervalo de operación"
    "RMS temporal durante el intervalo de operación"
    "Máximo del equivalente trifásico en cualquier instante"
    "Informativa: la guía no proporciona límite en A pico"
    "Se conserva para comparar con análisis anteriores; no decide cumplimiento"
    "RMS global; puede ocultar transitorios, por eso no se usa como máximo"
    "RMS global; puede ocultar transitorios, por eso no se usa como máximo"
    "RMS global; puede ocultar transitorios, por eso no se usa como máximo"
    "Torque del eje motor, no el torque de salida de la caja"
    "Valor al final del intervalo de operación"
    ];

tabla_diagnostico = table(VariableDiag,ValorDiag,UnidadDiag,ComentarioDiag, ...
    'VariableNames',{'Variable','Valor','Unidad','Comentario'});

%% Datos derivados para gráficos y análisis posteriores
datos = struct;
datos.t = t;
datos.theta_m = theta_m;
datos.theta_m_ref = theta_ref;
datos.theta_m_est = theta_est;
datos.q = q;
datos.q_ref = q_ref;
datos.q_est = q_est;
datos.e_q = e_q;
datos.e_theta_obs = e_theta_obs;
datos.e_q_obs = e_q_obs;
datos.wm = wm;
datos.wm_ref = wm_ref;
datos.wm_est = wm_est;
datos.wl = wl;
datos.wl_ref = wl_ref;
datos.wl_est = wl_est;
datos.alpha_l = alpha_l;
datos.alpha_l_ref = alpha_l_ref;
datos.e_w_obs = e_w_obs;
datos.e_wl_obs = e_wl_obs;
datos.Tq = Tq;
datos.Tm = Tm;
datos.Ts = Ts;
datos.Tld = Tld_t;
datos.t_iabc = ti;
datos.Iabc = Iabc;
datos.irms_fase = irms_fase;
datos.i_rms_equiv = i_rms_equiv;
datos.i_rms_equiv_acel_max = i_rms_equiv_acel_max;
datos.i_rms_equiv_ciclo_max = i_rms_equiv_ciclo_max;
datos.i_pico = i_pico;
datos.i_rms_20ms_fases = i_rms_20ms_fases; % compatibilidad
datos.i_rms_20ms_max = i_rms_20ms_max;     % compatibilidad
datos.t_vabc = tv;
datos.Vabc = Vabc;
datos.Vll = Vll;
datos.vll_rms = vll_rms;
datos.vll_rms_equiv = vll_rms_equiv;
datos.vll_rms_equiv_max = vll_rms_equiv_max;
datos.vfase_pico = vfase_pico;
datos.fe_est = fe_est;
datos.intervalo_operacion = intervalo_operacion;
datos.intervalos_aceleracion = intervalos_aceleracion;
datos.mask_operacion_t = mask_op_t;
datos.mask_operacion_i = mask_op_i;
datos.mask_operacion_v = mask_op_v;
datos.mask_aceleracion_i = mask_acel_i;
datos.limites = L;
end

function [theta_ts,theta_ref_ts,theta_est_ts,wm_ts,wm_ref_ts,wm_est_ts, ...
    Tm_ts,Ts_ts,iabc_ts,vabc_ts] = obtener_senales(out)
yout = out.yout;
theta_ts     = yout.getElement('theta_m').Values;
theta_ref_ts = yout.getElement('theta_m_ref').Values;
theta_est_ts = yout.getElement('theta_m_est').Values;
wm_ts        = yout.getElement('w_m').Values;
wm_ref_ts    = yout.getElement('w_m_ref').Values;
wm_est_ts    = yout.getElement('w_m_est').Values;
Tm_ts        = yout.getElement('T_m').Values;
Ts_ts        = yout.getElement('T_s').Values;
iabc_ts      = yout.getElement('i_abc').Values;
vabc_ts      = yout.getElement('v_abc_ref').Values;
end

function valor = obtener_opcion(opciones,nombre,valor_default)
if isfield(opciones,nombre) && ~isempty(opciones.(nombre))
    valor = opciones.(nombre);
else
    valor = valor_default;
end
end

function validar_intervalo(intervalo,nombre)
assert(isnumeric(intervalo) && numel(intervalo) == 2 && ...
    all(isfinite(intervalo)) && intervalo(2) > intervalo(1), ...
    '%s debe tener la forma [t_inicio t_fin], con t_fin > t_inicio.',nombre);
intervalo = intervalo(:).'; %#ok<NASGU>
end

function validar_intervalos(intervalos,nombre)
assert(isnumeric(intervalos) && size(intervalos,2) == 2 && ...
    all(isfinite(intervalos(:))) && all(intervalos(:,2) > intervalos(:,1)), ...
    '%s debe ser una matriz N x 2 con intervalos válidos.',nombre);
end

function intervalos = detectar_intervalos_aceleracion(t,alpha_ref,intervalo_op,umbral_rel)
assert(isscalar(umbral_rel) && umbral_rel > 0 && umbral_rel < 1, ...
    'umbral_aceleracion_rel debe estar entre 0 y 1.');
mask_op = mascara_intervalos(t,intervalo_op);
a = abs(alpha_ref);
a_max = max(a(mask_op));
if ~isfinite(a_max) || a_max <= eps
    intervalos = zeros(0,2);
    return;
end
umbral = max(umbral_rel*a_max,1e-9);
mask = mask_op & a >= umbral;
intervalos = intervalos_desde_mascara(t,mask);
end

function intervalos = intervalos_desde_mascara(t,mask)
mask = logical(mask(:));
t = t(:);
cambios = diff([false; mask; false]);
inicios = find(cambios == 1);
finales = find(cambios == -1)-1;
intervalos = [t(inicios) t(finales)];
end

function intervalos = expandir_y_recortar_intervalos(intervalos,margen,limites)
if isempty(intervalos)
    intervalos = zeros(0,2);
    return;
end
intervalos(:,1) = max(intervalos(:,1)-margen,limites(1));
intervalos(:,2) = min(intervalos(:,2)+margen,limites(2));
intervalos = sortrows(intervalos,1);

% Fusionar intervalos solapados después de aplicar el margen.
resultado = intervalos(1,:);
for k = 2:size(intervalos,1)
    if intervalos(k,1) <= resultado(end,2)
        resultado(end,2) = max(resultado(end,2),intervalos(k,2));
    else
        resultado(end+1,:) = intervalos(k,:); %#ok<AGROW>
    end
end
intervalos = resultado;
end

function mask = mascara_intervalos(t,intervalos)
t = t(:);
if isempty(intervalos)
    mask = false(size(t));
    return;
end
if isvector(intervalos) && numel(intervalos) == 2
    intervalos = reshape(intervalos,1,2);
end
mask = false(size(t));
for k = 1:size(intervalos,1)
    mask = mask | (t >= intervalos(k,1) & t <= intervalos(k,2));
end
end

function [Margen,Uso_pct,Cumple] = evaluar_limites(Valor,Limite)
Margen = Limite-Valor;
Uso_pct = 100*Valor./Limite;
Cumple = strings(size(Valor));
for k = 1:numel(Valor)
    if ~isfinite(Valor(k)) || ~isfinite(Limite(k))
        Margen(k) = NaN;
        Uso_pct(k) = NaN;
        Cumple(k) = "No evaluado";
    elseif Valor(k) <= Limite(k)
        Cumple(k) = "Sí";
    else
        Cumple(k) = "NO";
    end
end
end

function duracion = duracion_condicion(t,condicion,mask)
t = t(:);
condicion = logical(condicion(:));
mask = logical(mask(:));
assert(numel(t) == numel(condicion) && numel(t) == numel(mask), ...
    'Dimensiones incompatibles al calcular duración de exceso.');
activo = condicion & mask;
if numel(t) < 2 || ~any(activo)
    duracion = 0;
else
    duracion = trapz(t,double(activo));
end
end

function y = interp_ts(ts,tq)
t = ts.Time(:);
y = vector_columna(ts.Data);
if numel(t) == numel(tq) && max(abs(t-tq)) <= 1e-12
    return;
end
y = interp1(t,y,tq,'linear','extrap');
end

function x = vector_columna(x)
x = squeeze(x);
if isrow(x), x = x.'; end
assert(isvector(x),'Se esperaba una señal escalar.');
x = x(:);
end

function X = matriz_por_tiempo(ts)
X = squeeze(ts.Data);
nt = numel(ts.Time);
if isvector(X), X = X(:); end
if size(X,1) ~= nt && size(X,2) == nt
    X = X.';
end
assert(size(X,1) == nt, ...
    'No se pudo asociar la señal %s con su tiempo.',ts.Name);
end

function dx = derivada_numerica(t,x)
t = t(:);
x = x(:);
if numel(t) < 3
    dx = zeros(size(x));
else
    dx = gradient(x,t);
end
end

function valor = rms_temporal(t,X)
t = t(:);
if isvector(X), X = X(:); end
assert(size(X,1) == numel(t), ...
    'Dimensiones incompatibles para RMS temporal.');
duracion = t(end)-t(1);
if duracion <= 0
    valor = sqrt(mean(X.^2,1,'omitnan'));
else
    valor = sqrt(trapz(t,X.^2,1)/duracion);
end
end

function [maximo,por_fase] = rms_movil_max(t,X,ventana)
% Métrica heredada, conservada solo para comparación con scripts previos.
t = t(:);
if isvector(X), X = X(:); end
if numel(t) < 2 || t(end) <= t(1)
    por_fase = max(abs(X),[],1);
    maximo = max(por_fase);
    return;
end

dt_obj = min(1e-4,max(median(diff(t)),1e-6));
tu = (t(1):dt_obj:t(end)).';
Xu = interp1(t,X,tu,'linear','extrap');
N = max(1,round(ventana/dt_obj));
energia = movmean(Xu.^2,N,1,'Endpoints','shrink');
rms_mov = sqrt(energia);
por_fase = max(rms_mov,[],1);
maximo = max(por_fase);
end
