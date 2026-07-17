%% PUNTO 4.a - PERFIL TRAPEZOIDAL DE VELOCIDAD
% 0--5 s:   q* sube de 0 a 2*pi rad
% 5--10 s:  q* permanece en 2*pi rad
% 10--15 s: q* regresa de 2*pi a 0 rad
%
% Se usa una referencia trapezoidal de velocidad, con aceleración
% y desaceleración constantes y limitadas.
% Las métricas distinguen régimen continuo, valores pico y corriente RMS
% equivalente durante los intervalos de aceleración, sin imponer una
% ventana arbitraria de 20 ms.
% Las figuras se generan con SignalPlotter y se exportan en PDF vectorial.

clearvars;
close all;
clc;

%% Organización de carpetas del proyecto
% projectdir/
% |-- sim/       <- este script, el modelo y los archivos de simulación
% |-- utils/     <- SignalPlotter.m
% `-- docs/img/resultados/
%     |-- punto_4a_original/
%     `-- punto_4a_trapecio_velocidad/

simdir = fileparts(mfilename('fullpath'));
projectdir = fileparts(simdir);
utilsdir = fullfile(projectdir,'utils');
resultadosdir = fullfile(projectdir,'docs','img','resultados');
configdir=fullfile(simdir,'config');

% Agrega al path la carpeta de simulación y la de utilidades.
addpath(simdir);
addpath(utilsdir);
addpath(configdir);
rehash path;
validar_signalplotter(utilsdir);

% Esta carpeta queda al mismo nivel que punto_4a_original.
resdir = fullfile(resultadosdir,'punto_4a_trapecio_velocidad');
figdir = resdir;
crear_carpeta(resdir);
plotter = SignalPlotter(figdir);

% La inicialización se busca primero dentro de sim y luego en la raíz.
cargar_inicializacion(projectdir,simdir,configdir);

modelo = "simulacion_sistema_completo";
archivo_modelo = fullfile(simdir,modelo+".slx");
assert(isfile(archivo_modelo),'No se encontró %s.',archivo_modelo);
load_system(archivo_modelo);

%% Planta nominal
ml_planta = ml_nom;
bl_planta = bl_nom;
[Jl_planta,kl_planta,Jeq_planta,beq_planta,kg_eq_planta] = ...
    parametros_planta(ml_planta,bl_planta,m,Lcm,Jcm,Ll,Jm,bm,r,g);

%% Referencias y perturbaciones
T_mov = 5.0;          % duración de cada desplazamiento [s]
t_espera = 5.0;       % permanencia en q = 2*pi [s]
t_stop = 2*T_mov + t_espera;
Delta_q = 2*pi;       % desplazamiento de la carga [rad]
t_acc = 1.0;          % duración de aceleración y desaceleración [s]

assert(t_acc > 0 && 2*t_acc < T_mov, ...
    'Debe cumplirse 0 < t_acc < T_mov/2.');

% El área bajo el trapecio de velocidad debe ser Delta_q:
% Delta_q = w_max*(T_mov - t_acc).
w_max = Delta_q/(T_mov-t_acc);
alpha_max = w_max/t_acc;

% q_t queda nula: el bloque set_point integra w_t para generar q*.
ts_q = timeseries([0;0],[0;t_stop],'Name','q_t');
ts_q = setinterpmethod(ts_q,'zoh');

% Movimiento de ida: acelerar, velocidad constante y desacelerar.
% Movimiento de vuelta: perfil simétrico con velocidad negativa.
t_retorno = T_mov + t_espera;
t_w = [0; t_acc; T_mov-t_acc; T_mov; t_retorno; ...
       t_retorno+t_acc; t_stop-t_acc; t_stop];
w_ref = [0; w_max; w_max; 0; 0; -w_max; -w_max; 0];
ts_w = timeseries(w_ref,t_w,'Name','w_t');
ts_w = setinterpmethod(ts_w,'linear');

% Intervalos de aceleración/desaceleración usados para evaluar la corriente
% máxima de corta duración.
intervalos_aceleracion = [
    0,                  t_acc
    T_mov-t_acc,        T_mov
    t_retorno,          t_retorno+t_acc
    t_stop-t_acc,       t_stop
    ];

fprintf('Perfil trapezoidal de velocidad:\n');
fprintf('  w_max     = %.6f rad/s\n',w_max);
fprintf('  alpha_max = %.6f rad/s^2\n',alpha_max);

t_Tld = [0;t_stop];
Tld = [0;0];
ts_Tld = timeseries(Tld,t_Tld,'Name','T_ld');
ts_Tld = setinterpmethod(ts_Tld,'zoh');

ts_Tamb = timeseries([Tamb;Tamb],[0;t_stop],'Name','T_amb');
ts_Tamb = setinterpmethod(ts_Tamb,'zoh');

entrada = Simulink.SimulationData.Dataset;
entrada = entrada.addElement(ts_Tld,'T_ld');
entrada = entrada.addElement(ts_Tamb,'T_amb');
entrada = entrada.addElement(ts_q,'q_t');
entrada = entrada.addElement(ts_w,'w_t');

%% Simulación
in = Simulink.SimulationInput(modelo);
in = in.setExternalInput(entrada);
in = in.setModelParameter('StopTime',string(t_stop));
in = configurar_solver_robusto(in);
in = cargar_variables_planta(in,ml_planta,bl_planta,Jl_planta, ...
    kl_planta,Jeq_planta,beq_planta,kg_eq_planta);

disp('Ejecutando punto 4.a con perfil trapezoidal de velocidad...');
out = sim(in);
disp('Simulación terminada.');

% Verificación explícita de Outports 11 y 12.
out.yout.getElement('theta_m_est');
out.yout.getElement('w_m_est');

planta = struct( ...
    'r',r,'Jl',Jl_planta,'bl',bl_planta,'g',g,'kl',kl_planta,'Pp',Pp);

opciones_metricas = struct;
opciones_metricas.intervalo_operacion = [0 t_stop];
opciones_metricas.intervalos_aceleracion = intervalos_aceleracion;

% Se agrega un pequeño margen para incluir la respuesta dinámica del lazo
% inmediatamente después de cada cambio de aceleración. Este margen NO es
% una ventana RMS: la corriente se evalúa mediante su equivalente
% trifásico sqrt((ia^2+ib^2+ic^2)/3).
opciones_metricas.margen_aceleracion = 5e-3;

[tabla_limites,tabla_observador,datos,tabla_diagnostico] = ...
    calcular_metricas_operacion(out,planta,t_Tld,Tld,opciones_metricas);

%% Figuras PDF con SignalPlotter
[fig,ax] = plotter.plotTime(datos.t,[datos.q_ref datos.q datos.q_est], ...
    Title="Punto 4.a - Trapecio de velocidad: posición", ...
    XLabel="Tiempo [s]",YLabel="q [rad]", ...
    Legends={'q^*','q','q estimada'});
aplicar_estilos(ax,{'--','-',':'});
plotter.export(fig,'01_posicion_real_estimada.pdf');

[fig,ax] = plotter.plotTime(datos.t,[datos.wl_ref datos.wl datos.wl_est], ...
    Title="Punto 4.a - Trapecio de velocidad: velocidad", ...
    XLabel="Tiempo [s]",YLabel="\omega_l [rad/s]", ...
    Legends={'\omega_l^*','\omega_l','\omega_l estimada'});
aplicar_estilos(ax,{'--','-',':'});
plotter.export(fig,'02_velocidad_real_estimada.pdf');

[fig,ax] = plotter.plotTime(datos.t,[datos.alpha_l_ref datos.alpha_l], ...
    Title="Punto 4.a - Aceleración de la carga", ...
    XLabel="Tiempo [s]",YLabel="\alpha_l [rad/s^2]", ...
    Legends={'\alpha_l^*','\alpha_l'});
aplicar_estilos(ax,{'--','-'});
marcar_intervalos(ax,datos.intervalos_aceleracion);
plotter.export(fig,'02b_aceleracion_referencia_real.pdf');

[fig,ax] = plotter.plotTime(datos.t,[datos.e_q_obs datos.e_wl_obs], ...
    Title="Errores del observador reducido", ...
    XLabel="Tiempo [s]",YLabel="Error referido al eje de carga", ...
    Legends={'q-q estimada [rad]','\omega_l-\omega_l estimada [rad/s]'});
plotter.export(fig,'03_errores_observador.pdf');

[fig,ax] = plotter.plotTime(datos.t,datos.e_q, ...
    Title="Error de seguimiento de posición", ...
    XLabel="Tiempo [s]",YLabel="q^*-q [rad]", ...
    Legends={'e_q'});
plotter.export(fig,'04_error_seguimiento.pdf');

[fig,ax] = plotter.plotTime( ...
    datos.t, datos.Tm, ...
    Title="Torque electromagnético del motor", ...
    XLabel="Tiempo [s]", ...
    YLabel="T_m [N*m]", ...
    Legends={'T_m'});

plotter.export(fig, '05a_torque_motor.pdf');

[fig,ax] = plotter.plotTime( ...
    datos.t, datos.Tq, ...
    Title="Torque calculado en la salida de la caja reductora", ...
    XLabel="Tiempo [s]", ...
    YLabel="T_q [N*m]", ...
    Legends={'T_q calculado'});

hold(ax, 'on');
yline(ax, 17, ':', 'Nominal +17 N*m');
yline(ax, -17, ':', 'Nominal -17 N*m');
yline(ax, 45, '--', 'Límite pico +45 N*m');
yline(ax, -45, '--', 'Límite pico -45 N*m');
hold(ax, 'off');

plotter.export(fig, '05b_torque_salida.pdf');

Iqd0 = matriz_senal(out.yout.getElement('i_qd0s').Values);
t_iqd0 = out.yout.getElement('i_qd0s').Values.Time(:);
[fig,ax] = plotter.plotTime(t_iqd0,Iqd0, ...
    Title="Corrientes en coordenadas qd0", ...
    XLabel="Tiempo [s]",YLabel="Corriente [A]", ...
    Legends={'i_q','i_d','i_0'});
plotter.export(fig,'06_corrientes_qd0.pdf');

[fig,ax] = plotter.plotTime(datos.t_iabc,datos.Iabc, ...
    Title="Corrientes trifásicas", ...
    XLabel="Tiempo [s]",YLabel="Corriente [A]", ...
    Legends={'i_a','i_b','i_c'});
plotter.export(fig,'07_corrientes_abc.pdf');

[fig,ax] = plotter.plotTime(datos.t_iabc,datos.i_rms_equiv, ...
    Title="Corriente RMS equivalente trifásica", ...
    XLabel="Tiempo [s]",YLabel="I_{fase,rms,eq} [A]", ...
    Legends={'sqrt((i_a^2+i_b^2+i_c^2)/3)'});
hold(ax,'on');
yline(ax,0.4,':','Límite continuo 0.4 A RMS');
yline(ax,2.0,'--','Límite corto 2 A RMS');
marcar_intervalos(ax,datos.intervalos_aceleracion);
hold(ax,'off');
plotter.export(fig,'07b_corriente_rms_equivalente.pdf');

[fig,ax] = plotter.plotTime(datos.t_vabc,datos.Vabc, ...
    Title="Tensiones trifásicas solicitadas", ...
    XLabel="Tiempo [s]",YLabel="Tensión de fase [V]", ...
    Legends={'v_a^*','v_b^*','v_c^*'});
plotter.export(fig,'08_tensiones_abc.pdf');

[fig,ax] = plotter.plotTime(datos.t_vabc,datos.vll_rms_equiv, ...
    Title="Módulo RMS equivalente de tensión de línea", ...
    XLabel="Tiempo [s]",YLabel="V_{ll,rms,eq} [V]", ...
    Legends={'sqrt((V_{ab}^2+V_{bc}^2+V_{ca}^2)/3)'});
hold(ax,'on');
yline(ax,30,':','Tensión nominal del motor: 30 V RMS');
yline(ax,48,'--','Máximo del inversor: 48 V RMS');
hold(ax,'off');
plotter.export(fig,'08b_tension_linea_rms_equivalente.pdf');

[fig,ax] = plotter.plotTime(datos.t,datos.Ts, ...
    Title="Temperatura del estator", ...
    XLabel="Tiempo [s]",YLabel="T_s [degC]", ...
    Legends={'T_s'});
plotter.export(fig,'09_temperatura.pdf');

% Ampliaciones útiles para visualizar las ondas trifásicas y los transitorios.
exportar_zoom(plotter,datos.t_iabc,datos.Iabc,[2.0 2.1], ...
    'Corrientes trifásicas - detalle en régimen','Corriente [A]', ...
    {'i_a','i_b','i_c'},'10_corrientes_abc_zoom_regimen.pdf');
exportar_zoom(plotter,datos.t_iabc,datos.Iabc,[4.90 5.10], ...
    'Corrientes trifásicas - transición en t=5 s','Corriente [A]', ...
    {'i_a','i_b','i_c'},'11_corrientes_abc_zoom_transitorio.pdf');
exportar_zoom(plotter,datos.t_vabc,datos.Vabc,[4.90 5.10], ...
    'Tensiones trifásicas - transición en t=5 s','Tensión de fase [V]', ...
    {'v_a^*','v_b^*','v_c^*'},'12_tensiones_abc_zoom_transitorio.pdf');

%% Guardado de resultados
resultado_4a = struct;
resultado_4a.descripcion = 'Punto 4.a - perfil trapezoidal de velocidad';
resultado_4a.parametros_perfil = struct('Delta_q',Delta_q,'T_mov',T_mov, ...
    't_acc',t_acc,'w_max',w_max,'alpha_max',alpha_max);
resultado_4a.out = out;
resultado_4a.entrada = entrada;
resultado_4a.planta = planta;
resultado_4a.t_Tld = t_Tld;
resultado_4a.Tld = Tld;
resultado_4a.tabla_limites = tabla_limites;
resultado_4a.tabla_observador = tabla_observador;
resultado_4a.tabla_diagnostico = tabla_diagnostico;
resultado_4a.opciones_metricas = opciones_metricas;
resultado_4a.datos = datos;

save(fullfile(resdir,'resultado_punto_4a_trapecio_velocidad.mat'), ...
    'resultado_4a','-v7.3');
writetable(tabla_limites,fullfile(resdir,'limites_4a.csv'));
writetable(tabla_observador,fullfile(resdir,'metricas_observador_4a.csv'));
writetable(tabla_diagnostico,fullfile(resdir,'metricas_diagnostico_4a.csv'));

disp(' ');
disp('Verificación formal de límites:'); disp(tabla_limites);
disp('Métricas del observador:'); disp(tabla_observador);
disp('Métricas complementarias de diagnóstico:'); disp(tabla_diagnostico);
fprintf('\nResultados guardados en:\n%s\n',resdir);

%% Funciones locales
function validar_signalplotter(utilsdir)
archivo_clase = fullfile(utilsdir,'SignalPlotter.m');
archivo_clase_alt = fullfile(utilsdir,'signalPlotter.m');

assert(isfile(archivo_clase) || isfile(archivo_clase_alt), ...
    'No se encontró SignalPlotter.m en la carpeta utils: %s',utilsdir);

assert(exist('SignalPlotter','class') == 8, ...
    ['MATLAB encontró la carpeta utils, pero no reconoce la clase ' ...
     'SignalPlotter. Verifique que el nombre de la clase y del archivo coincidan.']);
end

function aplicar_estilos(ax,estilos)
lineas = findobj(ax,'Type','Line','HandleVisibility','on');
lineas = flipud(lineas);
for k = 1:min(numel(lineas),numel(estilos))
    lineas(k).LineStyle = estilos{k};
end
end

function exportar_zoom(plotter,t,Y,ventana,titulo,ylabeltxt,legendas,nombre)
mask = t >= ventana(1) & t <= ventana(2);
if nnz(mask) < 2, return; end
[fig,ax] = plotter.plotTime(t(mask),Y(mask,:), ...
    Title=titulo,XLabel="Tiempo [s]",YLabel=ylabeltxt,Legends=legendas);
plotter.export(fig,nombre);
end

function X = matriz_senal(ts)
X = squeeze(ts.Data);
nt = numel(ts.Time);
if isvector(X), X = X(:); end
if size(X,1) ~= nt && size(X,2) == nt, X = X.'; end
assert(size(X,1) == nt,'Dimensiones incompatibles en %s.',ts.Name);
end

function marcar_intervalos(ax,intervalos)
% Marca inicio y fin de cada intervalo de aceleración sin sombrear la figura.
for k = 1:size(intervalos,1)
    xline(ax,intervalos(k,1),':','Aceleración');
    xline(ax,intervalos(k,2),':');
end
end

function cargar_inicializacion(projectdir,simdir,configdir)
candidatos = { ...
    fullfile(configdir,'iniciar_proyecto.m'),...
    fullfile(configdir,'init_sistema_completo.m'),...
    fullfile(simdir,'iniciar_proyecto.m'), ...
    fullfile(simdir,'init_sistema_completo.m'), ...
    fullfile(projectdir,'config','iniciar_proyecto.m'), ...
    fullfile(projectdir,'config','init_sistema_completo.m'), ...
    fullfile(projectdir,'iniciar_proyecto.m'), ...
    fullfile(projectdir,'init_sistema_completo.m')};

initfile = '';
for k = 1:numel(candidatos)
    if isfile(candidatos{k})
        initfile = candidatos{k};
        break;
    end
end

if isempty(initfile)
    error(['No se encontró el script de inicialización. Se buscó en sim, ' ...
           'en config y en la raíz del proyecto.']);
end

run(initfile);

nombres = who;
excluir = {'projectdir','simdir','candidatos','initfile', ...
    'k','nombres','excluir','nombre'};

for k = 1:numel(nombres)
    nombre = nombres{k};
    if ~ismember(nombre,excluir)
        assignin('caller',nombre,eval(nombre));
    end
end
end

function [Jl,kl,Jeq,beq,kg] = parametros_planta(ml,bl,m,Lcm,Jcm,Ll,Jm,bm,r,g)
Jl = (m*Lcm^2+Jcm)+ml*Ll^2;
kl = m*Lcm+ml*Ll;
Jeq = Jm+Jl/r^2;
beq = bm+bl/r^2;
kg = g*kl/r;
end

function in = cargar_variables_planta(in,ml,bl,Jl,kl,Jeq,beq,kg)
in = in.setVariable('ml_planta',ml);
in = in.setVariable('bl_planta',bl);
in = in.setVariable('Jl_planta',Jl);
in = in.setVariable('kl_planta',kl);
in = in.setVariable('Jeq_planta',Jeq);
in = in.setVariable('beq_planta',beq);
in = in.setVariable('kg_eq_planta',kg);
end

function in = configurar_solver_robusto(in)
in = in.setModelParameter( ...
    'SolverType','Variable-step','Solver','ode23tb', ...
    'InitialStep','1e-8','MaxStep','1e-4','MinStep','auto', ...
    'RelTol','1e-3','AbsTol','auto','SolverResetMethod','Robust');
end

function crear_carpeta(p)
if ~isfolder(p), mkdir(p); end
end
