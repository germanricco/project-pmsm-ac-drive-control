%% Comparación entre los modelos no lineal y LTI
% No se utiliza clear ni clear all porque los parámetros de la planta
% ya deben encontrarse cargados en el workspace.

close all;
clc;

%% Configuración visual del eje temporal
dtMajor = 0.1;      % separación de marcas principales en eje x [s]
dtMinor = 0.05;     % separación de marcas menores en eje x [s]

showChangeLines = false;  % true: dibuja líneas verticales donde cambian v_qs y T_ld

%% Agregar la carpeta utils al path
% Este script está en /sim y SignalPlotter.m está en /utils.
scriptFolder = fileparts(mfilename('fullpath'));
utilsFolder  = fullfile(scriptFolder, '..', 'utils');

addpath(utilsFolder);

%% Verificar que exista el resultado de la simulación
if ~exist('out', 'var')
    error(['No existe la variable "out" en el workspace. ', ...
           'Ejecuta primero el modelo de Simulink.']);
end

%% Carpeta de exportación
exportFolder = fullfile(scriptFolder, '..', 'docs/img', ...
                        'comparacion_nl_lti');

plotter = SignalPlotter(exportFolder);

%% Extraer las señales de Simulink
[tThetaNL, thetaNL] = getSimSignal(out.theta_m_nl);
[tThetaLTI, thetaLTI] = getSimSignal(out.theta_m_lti);

[tWNL, wNL] = getSimSignal(out.w_m_nl);
[tWLTI, wLTI] = getSimSignal(out.w_m_lti);

[tTsNL, TsNL] = getSimSignal(out.T_s_nl);
[tTsLTI, TsLTI] = getSimSignal(out.T_s_lti);

[tTmNL, TmNL] = getSimSignal(out.T_m_nl);
[tTmLTI, TmLTI] = getSimSignal(out.T_m_lti);

%% Entradas aplicadas
[tVqs, vqs] = getSimSignal(out.v_qs_sim);
[tTld, Tld] = getSimSignal(out.T_ld_sim);

%% Detectar instantes donde cambian las entradas
changeTimesVqs = detectChangeTimes(tVqs, vqs);
changeTimesTld = detectChangeTimes(tTld, Tld);

changeTimes = unique([changeTimesVqs; changeTimesTld]);
changeTimes = changeTimes(:);

%% Comprobar y unificar los vectores de tiempo
% Si ambos modelos usan el mismo solver y reciben las mismas entradas,
% normalmente sus vectores temporales serán iguales.
%
% Si no coinciden exactamente, se interpola la señal LTI sobre el tiempo
% del modelo no lineal exclusivamente para poder representarlas juntas.

thetaLTI = alignSignal(tThetaLTI, thetaLTI, tThetaNL);
wLTI     = alignSignal(tWLTI,     wLTI,     tWNL);
TsLTI    = alignSignal(tTsLTI,    TsLTI,    tTsNL);
TmLTI    = alignSignal(tTmLTI,    TmLTI,    tTmNL);

%% 1. Posición angular del rotor
[figTheta, ~] = plotter.plotTime( ...
    tThetaNL, ...
    [thetaNL, thetaLTI], ...
    Title   = "Comparación de posición angular", ...
    XLabel  = "Tiempo (s)", ...
    YLabel  = "\theta_m (rad)", ...
    Legends = {"Modelo no lineal", "Modelo LTI"});

formatTimeAxis(figTheta, tThetaNL, dtMajor, dtMinor);

if showChangeLines
    addChangeLines(figTheta, changeTimes);
end

plotter.export(figTheta, "comparacion_theta_m.pdf");
plotter.export(figTheta, "comparacion_theta_m.png");

%% 2. Velocidad angular del rotor
[figW, ~] = plotter.plotTime( ...
    tWNL, ...
    [wNL, wLTI], ...
    Title   = "Comparación de velocidad angular", ...
    XLabel  = "Tiempo (s)", ...
    YLabel  = "\omega_m (rad/s)", ...
    Legends = {"Modelo no lineal", "Modelo LTI"});

formatTimeAxis(figW, tWNL, dtMajor, dtMinor);

if showChangeLines
    addChangeLines(figW, changeTimes);
end

plotter.export(figW, "comparacion_w_m.pdf");
plotter.export(figW, "comparacion_w_m.png");

%% 3. Temperatura del estator
[figTs, ~] = plotter.plotTime( ...
    tTsNL, ...
    [TsNL, TsLTI], ...
    Title   = "Comparación de temperatura del estator", ...
    XLabel  = "Tiempo (s)", ...
    YLabel  = "T_s (°C)", ...
    Legends = {"Modelo no lineal", "Modelo LTI"});

formatTimeAxis(figTs, tTsNL, dtMajor, dtMinor);

if showChangeLines
    addChangeLines(figTs, changeTimes);
end

plotter.export(figTs, "comparacion_T_s.pdf");
plotter.export(figTs, "comparacion_T_s.png");

%% 4. Torque electromagnético
[figTm, ~] = plotter.plotTime( ...
    tTmNL, ...
    [TmNL, TmLTI], ...
    Title   = "Comparación de torque electromagnético", ...
    XLabel  = "Tiempo (s)", ...
    YLabel  = "T_m (N·m)", ...
    Legends = {"Modelo no lineal", "Modelo LTI"});

formatTimeAxis(figTm, tTmNL, dtMajor, dtMinor);

if showChangeLines
    addChangeLines(figTm, changeTimes);
end

plotter.export(figTm, "comparacion_T_m.pdf");
plotter.export(figTm, "comparacion_T_m.png");

%% 5. Entradas aplicadas
% Como estas entradas fueron construidas por tramos, conviene usar stairs.
% Se utiliza plotTimeStairs si ya agregaron ese método a SignalPlotter.

if ismethod(plotter, 'plotTimeStairs')

    [figVqs, ~] = plotter.plotTimeStairs( ...
        tVqs, ...
        vqs, ...
        Title   = "Tensión aplicada en el eje q", ...
        XLabel  = "Tiempo (s)", ...
        YLabel  = "v_{qs} (V)", ...
        Legends = {"v_{qs}"});

    [figTld, ~] = plotter.plotTimeStairs( ...
        tTld, ...
        Tld, ...
        Title   = "Torque de perturbación aplicado", ...
        XLabel  = "Tiempo (s)", ...
        YLabel  = "T_{ld} (N·m)", ...
        Legends = {"T_{ld}"});

else
    warning(['SignalPlotter no contiene el método plotTimeStairs. ', ...
             'Las entradas se graficarán con plotTime.']);

    [figVqs, ~] = plotter.plotTime( ...
        tVqs, ...
        vqs, ...
        Title   = "Tensión aplicada en el eje q", ...
        XLabel  = "Tiempo (s)", ...
        YLabel  = "v_{qs} (V)", ...
        Legends = {"v_{qs}"});

    [figTld, ~] = plotter.plotTime( ...
        tTld, ...
        Tld, ...
        Title   = "Torque de perturbación aplicado", ...
        XLabel  = "Tiempo (s)", ...
        YLabel  = "T_{ld} (N·m)", ...
        Legends = {"T_{ld}"});
end

formatTimeAxis(figVqs, tVqs, dtMajor, dtMinor);
formatTimeAxis(figTld, tTld, dtMajor, dtMinor);

if showChangeLines
    addChangeLines(figVqs, changeTimesVqs);
    addChangeLines(figTld, changeTimesTld);
end

plotter.export(figVqs, "entrada_v_qs.pdf");
plotter.export(figVqs, "entrada_v_qs.png");

plotter.export(figTld, "entrada_T_ld.pdf");
plotter.export(figTld, "entrada_T_ld.png");

disp("Gráficos generados y exportados correctamente.");


%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

function [t, y] = getSimSignal(signal)
%GETSIMSIGNAL Extrae tiempo y datos de los formatos habituales de Simulink:
%   - timeseries
%   - Simulink.Timeseries
%   - estructura con time/signals
%   - timetable
%   - DatasetElement con propiedad Values

    % DatasetElement
    if isprop(signal, 'Values')
        signal = signal.Values;
    end

    % Timeseries
    if isa(signal, 'timeseries')
        t = signal.Time;
        y = signal.Data;

    % Estructura "Structure With Time"
    elseif isstruct(signal) && ...
            isfield(signal, 'time') && ...
            isfield(signal, 'signals')

        t = signal.time;
        y = signal.signals.values;

    % Timetable
    elseif istimetable(signal)
        t = seconds(signal.Properties.RowTimes ...
                    - signal.Properties.RowTimes(1));
        y = signal.Variables;

    else
        error(['Formato de señal no reconocido. Configura los bloques ', ...
               '"To Workspace" como Timeseries o Structure With Time.']);
    end

    t = squeeze(t);
    y = squeeze(y);

    t = t(:);

    % Para señales escalares, garantizar vector columna.
    if isvector(y)
        y = y(:);
    end

    if size(y, 1) ~= numel(t)
        if size(y, 2) == numel(t)
            y = y.';
        else
            error('El número de muestras no coincide con el vector temporal.');
        end
    end
end


function yAligned = alignSignal(tOriginal, yOriginal, tReference)
%ALIGNSIGNAL Alinea una señal con un vector temporal de referencia.

    tOriginal  = tOriginal(:);
    tReference = tReference(:);

    sameLength = numel(tOriginal) == numel(tReference);

    if sameLength
        tolerance = 100 * eps(max(1, max(abs(tReference))));
        sameTime = all(abs(tOriginal - tReference) <= tolerance);
    else
        sameTime = false;
    end

    if sameTime
        yAligned = yOriginal;
    else
        yAligned = interp1( ...
            tOriginal, ...
            yOriginal, ...
            tReference, ...
            'linear', ...
            'extrap');
    end
end


function formatTimeAxis(fig, t, dtMajor, dtMinor)
%FORMATTIMEAXIS Mejora la lectura del eje temporal.
%
% dtMajor: separación entre marcas principales del eje x.
% dtMinor: separación sugerida entre marcas menores del eje x.

    figure(fig);

    ax = gca;
    t = t(:);

    tMin = floor(min(t)/dtMajor) * dtMajor;
    tMax = ceil(max(t)/dtMajor)  * dtMajor;

    xlim(ax, [tMin tMax]);
    xticks(ax, tMin:dtMajor:tMax);

    grid(ax, 'on');

    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    ax.XMinorGrid = 'on';

    % Algunas versiones de MATLAB no permiten definir MinorTickValues.
    % Por eso se deja la grilla menor activada sin forzar valores.
    if isprop(ax, 'XAxis') && isprop(ax.XAxis, 'MinorTickValues')
        ax.XAxis.MinorTickValues = tMin:dtMinor:tMax;
    end

    % Mejora visual de los números del eje x.
    ax.XAxis.Exponent = 0;
end


function changeTimes = detectChangeTimes(t, y)
%DETECTCHANGETIMES Detecta los instantes donde cambia una señal por tramos.

    t = t(:);

    if isvector(y)
        y = y(:);
    end

    % Si la señal tiene varias columnas, se detecta cambio en cualquiera.
    dy = diff(y, 1, 1);

    if isvector(dy)
        idx = find(abs(dy) > 1e-12);
    else
        idx = find(any(abs(dy) > 1e-12, 2));
    end

    changeTimes = t(idx + 1);

    % Evitar marcar el instante inicial si aparece por algún artefacto.
    if ~isempty(changeTimes)
        changeTimes = changeTimes(changeTimes > min(t));
    end
end


function addChangeLines(fig, changeTimes)
%ADDCHANGELINES Agrega líneas verticales en los cambios de entrada.

    if isempty(changeTimes)
        return;
    end

    figure(fig);
    ax = gca;

    holdState = ishold(ax);
    hold(ax, 'on');

    yl = ylim(ax);

    for k = 1:numel(changeTimes)
        tk = changeTimes(k);

        % Usar xline si está disponible.
        if exist('xline', 'file') == 2
            xline(ax, tk, '--', ...
                'HandleVisibility', 'off', ...
                'LineWidth', 0.8);
        else
            line(ax, [tk tk], yl, ...
                'LineStyle', '--', ...
                'LineWidth', 0.8, ...
                'HandleVisibility', 'off');
        end
    end

    ylim(ax, yl);

    if ~holdState
        hold(ax, 'off');
    end
end