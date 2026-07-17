%% PUNTO 4.b - RECHAZO A PERTURBACIÓN Y ROBUSTEZ MECÁNICA
% q*=0 y escalón T_ld=+5 N*m en t=6.5 s.
% Se comparan la planta nominal y los extremos de carga/fricción.
% El controlador, compensador y observador permanecen nominales.

clearvars;
close all;
clc;

%% Organización de carpetas del proyecto
% projectdir/
% |-- sim/       <- este script, el modelo y los archivos de simulación
% |-- utils/     <- SignalPlotter.m
% `-- docs/img/resultados/
%     |-- punto_4a_original/
%     `-- punto_4b_perturbacion/

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
resdir = fullfile(resultadosdir,'punto_4b_perturbacion');
figdir = resdir;
crear_carpeta(resdir);
plotter = SignalPlotter(figdir);

% La inicialización se busca primero dentro de sim y luego en la raíz.
cargar_inicializacion(projectdir,simdir,configdir);

modelo = "simulacion_sistema_completo";
archivo_modelo = fullfile(simdir,modelo+".slx");
assert(isfile(archivo_modelo),'No se encontró %s.',archivo_modelo);
load_system(archivo_modelo);

%% Entradas
t_stop = 15;
t_escalon = 6.5;
t_rampa_escalon = 1e-6; % regularización numérica de 1 microsegundo

ts_q = timeseries([0;0],[0;t_stop],'Name','q_t');
ts_q = setinterpmethod(ts_q,'zoh');
ts_w = timeseries([0;0],[0;t_stop],'Name','w_t');
ts_w = setinterpmethod(ts_w,'zoh');

%% Perturbación aplicada
% Se evita la ambigüedad del ZOH usando una rampa de 1 us. Así se garantiza:
%   T_ld = 0 N*m para t < 6.5 s
%   T_ld = 5 N*m para t >= 6.5 s
t_Tld = [0; t_escalon-t_rampa_escalon; t_escalon; t_stop];
Tld   = [0; 0; 5; 5];
ts_Tld = timeseries(Tld,t_Tld,'Name','T_ld');
ts_Tld = setinterpmethod(ts_Tld,'linear');

% Vector independiente solo para graficar el escalón ideal.
t_Tld_plot = [0; t_escalon; t_stop];
Tld_plot   = [0; 5; 5];

ts_Tamb = timeseries([Tamb;Tamb],[0;t_stop],'Name','T_amb');
ts_Tamb = setinterpmethod(ts_Tamb,'zoh');

entrada = Simulink.SimulationData.Dataset;
entrada = entrada.addElement(ts_Tld,'T_ld');
entrada = entrada.addElement(ts_Tamb,'T_amb');
entrada = entrada.addElement(ts_q,'q_t');
entrada = entrada.addElement(ts_w,'w_t');

%% Casos de planta
nombres = [
    "Nominal"
    "Sin carga - fricción mínima"
    "Sin carga - fricción máxima"
    "Carga máxima - fricción mínima"
    "Carga máxima - fricción máxima"
    ];
ml_casos = [ml_nom;ml_min;ml_min;ml_max;ml_max];
bl_casos = [bl_nom;bl_min;bl_max;bl_min;bl_max];
ncasos = numel(nombres);
resultados_4b = repmat(struct,ncasos,1);

for k = 1:ncasos
    fprintf('\nSimulando caso %d/%d: %s\n',k,ncasos,nombres(k));
    ml_planta = ml_casos(k);
    bl_planta = bl_casos(k);
    [Jl_planta,kl_planta,Jeq_planta,beq_planta,kg_eq_planta] = ...
        parametros_planta(ml_planta,bl_planta,m,Lcm,Jcm,Ll,Jm,bm,r,g);

    in = Simulink.SimulationInput(modelo);
    in = in.setExternalInput(entrada);
    in = in.setModelParameter('StopTime',string(t_stop));
    in = configurar_solver_robusto(in);
    in = cargar_variables_planta(in,ml_planta,bl_planta,Jl_planta, ...
        kl_planta,Jeq_planta,beq_planta,kg_eq_planta);

    out = sim(in);
    out.yout.getElement('theta_m_est');
    out.yout.getElement('w_m_est');

    planta = struct('r',r,'Jl',Jl_planta,'bl',bl_planta,'g',g,'kl',kl_planta);
    [tabla_limites,tabla_observador,datos] = ...
        calcular_metricas_operacion(out,planta,t_Tld,Tld);

    despues = datos.t >= t_escalon;
    final = datos.t >= t_stop-0.5;
    resultados_4b(k).nombre = nombres(k);
    resultados_4b(k).ml = ml_planta;
    resultados_4b(k).bl = bl_planta;
    resultados_4b(k).planta = planta;
    resultados_4b(k).out = out;
    resultados_4b(k).datos = datos;
    resultados_4b(k).tabla_limites = tabla_limites;
    resultados_4b(k).tabla_observador = tabla_observador;
    resultados_4b(k).desvio_max_posicion = max(abs(datos.q(despues)));
    resultados_4b(k).error_seguimiento_final = mean(datos.e_q(final),'omitnan');
    resultados_4b(k).error_obs_pos_max = max(abs(datos.e_q_obs(despues)));
    resultados_4b(k).error_obs_vel_max = max(abs(datos.e_wl_obs(despues)));
end

%% Tabla resumen
Caso = nombres;
ml_kg = ml_casos;
bl_Nms = bl_casos;
DesvioMaxPosicion = zeros(ncasos,1);
ErrorSeguimientoFinal = zeros(ncasos,1);
ErrorObsPosMax = zeros(ncasos,1);
ErrorObsVelMax = zeros(ncasos,1);
for k = 1:ncasos
    DesvioMaxPosicion(k) = resultados_4b(k).desvio_max_posicion;
    ErrorSeguimientoFinal(k) = resultados_4b(k).error_seguimiento_final;
    ErrorObsPosMax(k) = resultados_4b(k).error_obs_pos_max;
    ErrorObsVelMax(k) = resultados_4b(k).error_obs_vel_max;
end
resumen_rechazo = table(Caso,ml_kg,bl_Nms,DesvioMaxPosicion, ...
    ErrorSeguimientoFinal,ErrorObsPosMax,ErrorObsVelMax);
disp('Resumen de rechazo y observación:');
disp(resumen_rechazo);

%% Figuras comparativas PDF
[t_comp,Q] = matriz_casos(resultados_4b,'q');
[fig,ax] = plotter.plotTime(t_comp,Q, ...
    Title="Punto 4.b - Desviación de posición ante el escalón", ...
    XLabel="Tiempo [s]",YLabel="q [rad]",Legends=cellstr(nombres));
plotter.export(fig,'01_posicion_real_casos.pdf');

[~,Epos] = matriz_casos(resultados_4b,'e_q_obs');
[fig,ax] = plotter.plotTime(t_comp,Epos, ...
    Title="Error de posición del observador ante perturbación", ...
    XLabel="Tiempo [s]",YLabel="q-q estimada [rad]", ...
    Legends=cellstr(nombres));
plotter.export(fig,'02_error_observador_posicion.pdf');

[~,W] = matriz_casos(resultados_4b,'wl');
[fig,ax] = plotter.plotTime(t_comp,W, ...
    Title="Velocidad de carga ante perturbación", ...
    XLabel="Tiempo [s]",YLabel="\omega_l [rad/s]",Legends=cellstr(nombres));
plotter.export(fig,'03_velocidad_real_casos.pdf');

[~,Evel] = matriz_casos(resultados_4b,'e_wl_obs');
[fig,ax] = plotter.plotTime(t_comp,Evel, ...
    Title="Error de velocidad del observador ante perturbación", ...
    XLabel="Tiempo [s]",YLabel="\omega_l - \omega_{l,est} [rad/s]", ...
    Legends=cellstr(nombres));
plotter.export(fig,'04_error_observador_velocidad.pdf');

[fig,ax] = plotter.plotTimeStairs(t_Tld_plot,Tld_plot, ...
    Title="Escalón de perturbación por contacto", ...
    XLabel="Tiempo [s]",YLabel="T_{ld} [N*m]",Legends={'T_{ld}'});
plotter.export(fig,'05_perturbacion.pdf');

% Caso nominal: comparación explícita de referencia, valor real y estimado.
% Se conservan las figuras en el eje de la carga para comparar con los
% demás casos de incertidumbre mecánica.
d0 = resultados_4b(1).datos;

%% Verificación de equilibrio antes de la perturbación
idx_pre = d0.t >= 0.5 & d0.t <= t_escalon-0.1;
max_theta_pre = max(abs(d0.theta_m(idx_pre)));
max_wm_pre = max(abs(d0.wm(idx_pre)));
fprintf('Antes del escalón: max|theta_m| = %.3e rad, max|w_m| = %.3e rad/s\n', ...
    max_theta_pre,max_wm_pre);
if max_theta_pre > 1e-8 || max_wm_pre > 1e-8
    warning(['El sistema no permanece exactamente en reposo antes del escalón. ' ...
        'Revise la señal T_ld aplicada o las condiciones iniciales.']);
end

[fig,ax] = plotter.plotTime(d0.t,[d0.q_ref d0.q d0.q_est], ...
    Title="Caso nominal - posición de carga real, estimada y referencia", ...
    XLabel="Tiempo [s]",YLabel="q [rad]", ...
    Legends={'q^*','q','q_{est}'});
aplicar_estilos(ax,{'--','-',':'});
plotter.export(fig,'06_nominal_posicion_carga_real_estimada.pdf');

[fig,ax] = plotter.plotTime(d0.t,[d0.wl_ref d0.wl d0.wl_est], ...
    Title="Caso nominal - velocidad de carga real, estimada y referencia", ...
    XLabel="Tiempo [s]",YLabel="\omega_l [rad/s]", ...
    Legends={'\omega_l^*','\omega_l','\omega_{l,est}'});
aplicar_estilos(ax,{'--','-',':'});
plotter.export(fig,'07_nominal_velocidad_carga_real_estimada.pdf');

%% Figuras del caso nominal equivalentes a las del informe Lage Tejo-Olguín
% En el informe de referencia se muestran las variables referidas al eje del
% motor y se amplía una ventana muy pequeña alrededor del escalón. Sin este
% zoom, el transitorio de unos pocos milisegundos queda comprimido dentro de
% los 15 s de simulación y parece una línea vertical.

t_zoom_ini = t_escalon - 2e-3;
t_zoom_fin = t_escalon + 25e-3;
idx_zoom = d0.t >= t_zoom_ini & d0.t <= t_zoom_fin;
assert(nnz(idx_zoom) >= 3, ...
    'No hay suficientes muestras en la ventana de zoom del escalón.');

t_zoom = d0.t(idx_zoom);

% Posición del motor: referencia, posición real y posición estimada.
[fig,ax] = plotter.plotTime(t_zoom, ...
    [d0.theta_m(idx_zoom), ...
     d0.theta_m_ref(idx_zoom), ...
     d0.theta_m_est(idx_zoom)], ...
    Title="Respuesta de posición del motor ante perturbación de carga", ...
    XLabel="Tiempo [s]",YLabel="\theta_m [rad]", ...
    Legends={'\theta_m','\theta_m^*','\theta_{m,est}'});
aplicar_estilos(ax,{'-','--',':'});
xlim(ax,[t_zoom_ini t_zoom_fin]);
plotter.export(fig,'08_nominal_posicion_motor_zoom.pdf');

% Error de posición del observador, referido al eje del motor.
[fig,ax] = plotter.plotTime(t_zoom,d0.e_theta_obs(idx_zoom), ...
    Title="Error de posición del observador ante perturbación", ...
    XLabel="Tiempo [s]", ...
    YLabel="\theta_m - \theta_{m,est} [rad]", ...
    Legends={'e_{\theta,obs}'});
xlim(ax,[t_zoom_ini t_zoom_fin]);
plotter.export(fig,'09_nominal_error_posicion_observador_zoom.pdf');

% Velocidad del motor: referencia, velocidad real y velocidad estimada.
[fig,ax] = plotter.plotTime(t_zoom, ...
    [d0.wm(idx_zoom), ...
     d0.wm_ref(idx_zoom), ...
     d0.wm_est(idx_zoom)], ...
    Title="Respuesta de velocidad del motor ante perturbación de carga", ...
    XLabel="Tiempo [s]",YLabel="\omega_m [rad/s]", ...
    Legends={'\omega_m','\omega_m^*','\omega_{m,est}'});
aplicar_estilos(ax,{'-','--',':'});
xlim(ax,[t_zoom_ini t_zoom_fin]);
plotter.export(fig,'10_nominal_velocidad_motor_zoom.pdf');

% Error de velocidad del observador, referido al eje del motor.
[fig,ax] = plotter.plotTime(t_zoom,d0.e_w_obs(idx_zoom), ...
    Title="Error de velocidad del observador ante perturbación", ...
    XLabel="Tiempo [s]", ...
    YLabel="\omega_m - \omega_{m,est} [rad/s]", ...
    Legends={'e_{\omega,obs}'});
xlim(ax,[t_zoom_ini t_zoom_fin]);
plotter.export(fig,'11_nominal_error_velocidad_observador_zoom.pdf');

% Evolución completa posterior al escalón. Esta figura permite ver si el
% error del observador se anula o conserva un valor estacionario.
idx_post = d0.t >= t_escalon;
[fig,ax] = plotter.plotTime(d0.t(idx_post),d0.e_theta_obs(idx_post), ...
    Title="Error de posición del observador después del escalón", ...
    XLabel="Tiempo [s]", ...
    YLabel="\theta_m - \theta_{m,est} [rad]", ...
    Legends={'e_{\theta,obs}'});
plotter.export(fig,'12_nominal_error_posicion_observador_postescalon.pdf');

%% Guardado
save(fullfile(resdir,'resultado_punto_4b.mat'), ...
    'resultados_4b','resumen_rechazo','entrada','t_Tld','Tld','-v7.3');
writetable(resumen_rechazo,fullfile(resdir,'resumen_rechazo_4b.csv'));
for k = 1:ncasos
    writetable(resultados_4b(k).tabla_observador, ...
        fullfile(resdir,sprintf('metricas_observador_caso_%02d.csv',k)));
    writetable(resultados_4b(k).tabla_limites, ...
        fullfile(resdir,sprintf('limites_caso_%02d.csv',k)));
end
fprintf('\nResultados guardados en:\n%s\n',resdir);

%% Funciones locales
function [t,Y] = matriz_casos(resultados,campo)
t = resultados(1).datos.t;
Y = zeros(numel(t),numel(resultados));
for k = 1:numel(resultados)
    tk = resultados(k).datos.t;
    yk = resultados(k).datos.(campo);
    Y(:,k) = interp1(tk,yk,t,'linear','extrap');
end
end

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
lineas = flipud(findobj(ax,'Type','Line','HandleVisibility','on'));
for k = 1:min(numel(lineas),numel(estilos)), lineas(k).LineStyle=estilos{k}; end
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

function [Jl,kl,Jeq,beq,kg]=parametros_planta(ml,bl,m,Lcm,Jcm,Ll,Jm,bm,r,g)
Jl=(m*Lcm^2+Jcm)+ml*Ll^2; kl=m*Lcm+ml*Ll;
Jeq=Jm+Jl/r^2; beq=bm+bl/r^2; kg=g*kl/r;
end

function in=cargar_variables_planta(in,ml,bl,Jl,kl,Jeq,beq,kg)
in=in.setVariable('ml_planta',ml); in=in.setVariable('bl_planta',bl);
in=in.setVariable('Jl_planta',Jl); in=in.setVariable('kl_planta',kl);
in=in.setVariable('Jeq_planta',Jeq); in=in.setVariable('beq_planta',beq);
in=in.setVariable('kg_eq_planta',kg);
end

function in=configurar_solver_robusto(in)
in=in.setModelParameter('SolverType','Variable-step','Solver','ode23tb', ...
    'InitialStep','1e-8','MaxStep','1e-4','MinStep','auto', ...
    'RelTol','1e-3','AbsTol','auto','SolverResetMethod','Robust');
end

function crear_carpeta(p), if ~isfolder(p), mkdir(p); end, end
