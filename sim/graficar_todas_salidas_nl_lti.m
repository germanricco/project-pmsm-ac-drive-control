%% Gráficas completas de los modelos no lineal (NL) y LTI
% -------------------------------------------------------------------------
% Requisitos:
%   1) Ejecutar primero el modelo de Simulink para obtener la variable out.
%   2) Este archivo debe estar en la carpeta /sim.
%   3) SignalPlotter.m debe estar en la carpeta /utils.
%
% Salidas esperadas en out:
%   Escalares:
%     theta_m_nl, w_m_nl, T_s_nl, T_m_nl
%     theta_m_lti, w_m_lti, T_s_lti, T_m_lti
%
%   Vectores de tres componentes:
%     i_qd0_nl, i_abc_nl, v_qd0_nl, v_abc_nl
%     i_qd0_lti, i_abc_lti, v_qd0_lti, v_abc_lti
%
%   Entradas opcionales:
%     v_qs_sim, T_ld_sim
%
% Convención adoptada para los vectores:
%   qd0 = [q, d, 0]
%   abc = [a, b, c]
%
% El script exporta EXCLUSIVAMENTE archivos PDF.
% No se usa clear ni clear all porque los parámetros de la planta deben
% permanecer cargados en el workspace.
% -------------------------------------------------------------------------

close all;
clc;

%% Configuración general

dtMajor = 0.1;          % separación de marcas principales del eje temporal [s]
dtMinor = 0.05;         % separación de marcas menores del eje temporal [s]
showChangeLines = false; % true: marca los cambios de v_qs y T_ld
closeAfterExport = true; % true: cierra cada figura luego de exportarla

% Orden de las componentes dentro de cada vector de Simulink.
% Modificar solamente si el Mux/Vector Concatenate usa otro orden.
qd0Order = [1 2 3];      % [q d 0]
abcOrder = [1 2 3];      % [a b c]

%% Rutas del proyecto

scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder)
    scriptFolder = pwd;
end

utilsFolder = fullfile(scriptFolder, '..', 'utils');
addpath(utilsFolder);

if exist('SignalPlotter', 'class') ~= 8
    error(['No se encontró SignalPlotter.m. Se esperaba encontrarlo en: ', ...
           utilsFolder]);
end

if ~exist('out', 'var')
    error(['No existe la variable "out" en el workspace. ', ...
           'Ejecuta primero el modelo de Simulink.']);
end

%% Carpeta de exportación

exportFolder = fullfile(scriptFolder, '..', 'docs', 'img', ...
                        'simulacion_nl_lti');

if ~isfolder(exportFolder)
    mkdir(exportFolder);
end

plotter = SignalPlotter(exportFolder);

%% Extraer señales escalares

[tThetaNL,  thetaNL]  = getSimSignal(getRequiredOutput(out, 'theta_m_nl'));
[tThetaLTI, thetaLTI] = getSimSignal(getRequiredOutput(out, 'theta_m_lti'));

[tWNL,  wNL]  = getSimSignal(getRequiredOutput(out, 'w_m_nl'));
[tWLTI, wLTI] = getSimSignal(getRequiredOutput(out, 'w_m_lti'));

[tTsNL,  TsNL]  = getSimSignal(getRequiredOutput(out, 'T_s_nl'));
[tTsLTI, TsLTI] = getSimSignal(getRequiredOutput(out, 'T_s_lti'));

[tTmNL,  TmNL]  = getSimSignal(getRequiredOutput(out, 'T_m_nl'));
[tTmLTI, TmLTI] = getSimSignal(getRequiredOutput(out, 'T_m_lti'));

%% Extraer señales vectoriales qd0 y abc

[tIqd0NL,  iqd0NL]  = getSimSignal(getRequiredOutput(out, 'i_qd0_nl'));
[tIqd0LTI, iqd0LTI] = getSimSignal(getRequiredOutput(out, 'i_qd0_lti'));

[tIabcNL,  iabcNL]  = getSimSignal(getRequiredOutput(out, 'i_abc_nl'));
[tIabcLTI, iabcLTI] = getSimSignal(getRequiredOutput(out, 'i_abc_lti'));

[tVqd0NL,  vqd0NL]  = getSimSignal(getRequiredOutput(out, 'v_qd0_nl'));
[tVqd0LTI, vqd0LTI] = getSimSignal(getRequiredOutput(out, 'v_qd0_lti'));

[tVabcNL,  vabcNL]  = getSimSignal(getRequiredOutput(out, 'v_abc_nl'));
[tVabcLTI, vabcLTI] = getSimSignal(getRequiredOutput(out, 'v_abc_lti'));

% Verificar cantidad de componentes.
requireComponents(iqd0NL,  3, 'out.i_qd0_nl');
requireComponents(iqd0LTI, 3, 'out.i_qd0_lti');
requireComponents(iabcNL,  3, 'out.i_abc_nl');
requireComponents(iabcLTI, 3, 'out.i_abc_lti');
requireComponents(vqd0NL,  3, 'out.v_qd0_nl');
requireComponents(vqd0LTI, 3, 'out.v_qd0_lti');
requireComponents(vabcNL,  3, 'out.v_abc_nl');
requireComponents(vabcLTI, 3, 'out.v_abc_lti');

% Aplicar el orden configurado al inicio.
iqd0NL  = iqd0NL(:,  qd0Order);
iqd0LTI = iqd0LTI(:, qd0Order);
vqd0NL  = vqd0NL(:,  qd0Order);
vqd0LTI = vqd0LTI(:, qd0Order);

iabcNL  = iabcNL(:,  abcOrder);
iabcLTI = iabcLTI(:, abcOrder);
vabcNL  = vabcNL(:,  abcOrder);
vabcLTI = vabcLTI(:, abcOrder);

%% Entradas aplicadas, si fueron registradas

hasVqs = false;
hasTld = false;
changeTimesVqs = [];
changeTimesTld = [];
changeTimes = [];

[hasVqs, vqsSignal] = tryGetOutput(out, 'v_qs_sim');
if hasVqs
    [tVqs, vqs] = getSimSignal(vqsSignal);
    changeTimesVqs = detectChangeTimes(tVqs, vqs);
end

[hasTld, TldSignal] = tryGetOutput(out, 'T_ld_sim');
if hasTld
    [tTld, Tld] = getSimSignal(TldSignal);
    changeTimesTld = detectChangeTimes(tTld, Tld);
end

changeTimes = unique([changeTimesVqs(:); changeTimesTld(:)]);

%% Alinear señales LTI con los tiempos NL para las comparaciones

% Escalares
thetaLTIonNL = alignSignal(tThetaLTI, thetaLTI, tThetaNL);
wLTIonNL     = alignSignal(tWLTI,     wLTI,     tWNL);
TsLTIonNL    = alignSignal(tTsLTI,    TsLTI,    tTsNL);
TmLTIonNL    = alignSignal(tTmLTI,    TmLTI,    tTmNL);

% Vectores
iqd0LTIonNL = alignSignal(tIqd0LTI, iqd0LTI, tIqd0NL);
iabcLTIonNL = alignSignal(tIabcLTI, iabcLTI, tIabcNL);
vqd0LTIonNL = alignSignal(tVqd0LTI, vqd0LTI, tVqd0NL);
vabcLTIonNL = alignSignal(tVabcLTI, vabcLTI, tVabcNL);

%% 1. Entradas aplicadas

if hasVqs
    exportInputPlot(plotter, tVqs, vqs, ...
        "Tensión de consigna aplicada en el eje q", ...
        "v_{qs}^{r} (V)", ...
        {"v_{qs}^{r}"}, ...
        "entrada_v_qs.pdf", ...
        dtMajor, dtMinor, showChangeLines, changeTimesVqs, closeAfterExport);
else
    warning('No se encontró out.v_qs_sim. Se omite la gráfica de entrada v_qs.');
end

if hasTld
    exportInputPlot(plotter, tTld, Tld, ...
        "Torque de perturbación aplicado", ...
        "T_{ld} (N·m)", ...
        {"T_{ld}"}, ...
        "entrada_T_ld.pdf", ...
        dtMajor, dtMinor, showChangeLines, changeTimesTld, closeAfterExport);
else
    warning('No se encontró out.T_ld_sim. Se omite la gráfica de entrada T_ld.');
end

%% 2. Comparación de variables mecánicas y térmicas

exportTimeComparison(plotter, tThetaNL, thetaNL, thetaLTIonNL, ...
    "Comparación de posición angular del rotor", ...
    "\theta_m (rad)", ...
    "comparacion_theta_m.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportTimeComparison(plotter, tWNL, wNL, wLTIonNL, ...
    "Comparación de velocidad angular del rotor", ...
    "\omega_m (rad/s)", ...
    "comparacion_w_m.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportTimeComparison(plotter, tTsNL, TsNL, TsLTIonNL, ...
    "Comparación de temperatura del estator", ...
    "T_s (°C)", ...
    "comparacion_T_s.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportTimeComparison(plotter, tTmNL, TmNL, TmLTIonNL, ...
    "Comparación de torque electromagnético", ...
    "T_m (N·m)", ...
    "comparacion_T_m.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

%% 3. Corrientes en coordenadas qd0

exportVectorTimePlot(plotter, tIqd0NL, iqd0NL, ...
    "Corrientes del modelo no lineal en coordenadas qd0", ...
    "Corriente (A)", ...
    {"i_{qs}^{r}", "i_{ds}^{r}", "i_{0s}"}, ...
    "corrientes_qd0_nl.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportVectorTimePlot(plotter, tIqd0LTI, iqd0LTI, ...
    "Corrientes del modelo LTI en coordenadas qd0", ...
    "Corriente (A)", ...
    {"i_{qs}^{r}", "i_{ds}^{r}", "i_{0s}"}, ...
    "corrientes_qd0_lti.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

qdCurrentTitles = {
    "Comparación de corriente en el eje q"
    "Comparación de corriente en el eje d"
    "Comparación de corriente homopolar"
};

qdCurrentYLabels = {
    "i_{qs}^{r} (A)"
    "i_{ds}^{r} (A)"
    "i_{0s} (A)"
};

qdCurrentFiles = {
    "comparacion_i_q.pdf"
    "comparacion_i_d.pdf"
    "comparacion_i_0.pdf"
};

for k = 1:3
    exportTimeComparison(plotter, tIqd0NL, ...
        iqd0NL(:, k), iqd0LTIonNL(:, k), ...
        qdCurrentTitles{k}, qdCurrentYLabels{k}, qdCurrentFiles{k}, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);
end

%% 4. Corrientes en coordenadas abc

exportVectorTimePlot(plotter, tIabcNL, iabcNL, ...
    "Corrientes trifásicas del modelo no lineal", ...
    "Corriente (A)", ...
    {"i_{as}", "i_{bs}", "i_{cs}"}, ...
    "corrientes_abc_nl.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportVectorTimePlot(plotter, tIabcLTI, iabcLTI, ...
    "Corrientes trifásicas del modelo LTI", ...
    "Corriente (A)", ...
    {"i_{as}", "i_{bs}", "i_{cs}"}, ...
    "corrientes_abc_lti.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

abcCurrentTitles = {
    "Comparación de corriente de fase a"
    "Comparación de corriente de fase b"
    "Comparación de corriente de fase c"
};

abcCurrentYLabels = {
    "i_{as} (A)"
    "i_{bs} (A)"
    "i_{cs} (A)"
};

abcCurrentFiles = {
    "comparacion_i_a.pdf"
    "comparacion_i_b.pdf"
    "comparacion_i_c.pdf"
};

for k = 1:3
    exportTimeComparison(plotter, tIabcNL, ...
        iabcNL(:, k), iabcLTIonNL(:, k), ...
        abcCurrentTitles{k}, abcCurrentYLabels{k}, abcCurrentFiles{k}, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);
end

%% 5. Tensiones en coordenadas qd0

exportVectorTimePlot(plotter, tVqd0NL, vqd0NL, ...
    "Tensiones del modelo no lineal en coordenadas qd0", ...
    "Tensión (V)", ...
    {"v_{qs}^{r}", "v_{ds}^{r}", "v_{0s}"}, ...
    "tensiones_qd0_nl.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportVectorTimePlot(plotter, tVqd0LTI, vqd0LTI, ...
    "Tensiones del modelo LTI en coordenadas qd0", ...
    "Tensión (V)", ...
    {"v_{qs}^{r}", "v_{ds}^{r}", "v_{0s}"}, ...
    "tensiones_qd0_lti.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

qdVoltageTitles = {
    "Comparación de tensión en el eje q"
    "Comparación de tensión en el eje d"
    "Comparación de tensión homopolar"
};

qdVoltageYLabels = {
    "v_{qs}^{r} (V)"
    "v_{ds}^{r} (V)"
    "v_{0s} (V)"
};

qdVoltageFiles = {
    "comparacion_v_q.pdf"
    "comparacion_v_d.pdf"
    "comparacion_v_0.pdf"
};

for k = 1:3
    exportTimeComparison(plotter, tVqd0NL, ...
        vqd0NL(:, k), vqd0LTIonNL(:, k), ...
        qdVoltageTitles{k}, qdVoltageYLabels{k}, qdVoltageFiles{k}, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);
end

%% 6. Tensiones en coordenadas abc

exportVectorTimePlot(plotter, tVabcNL, vabcNL, ...
    "Tensiones trifásicas del modelo no lineal", ...
    "Tensión (V)", ...
    {"v_{as}", "v_{bs}", "v_{cs}"}, ...
    "tensiones_abc_nl.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

exportVectorTimePlot(plotter, tVabcLTI, vabcLTI, ...
    "Tensiones trifásicas del modelo LTI", ...
    "Tensión (V)", ...
    {"v_{as}", "v_{bs}", "v_{cs}"}, ...
    "tensiones_abc_lti.pdf", ...
    dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);

abcVoltageTitles = {
    "Comparación de tensión de fase a"
    "Comparación de tensión de fase b"
    "Comparación de tensión de fase c"
};

abcVoltageYLabels = {
    "v_{as} (V)"
    "v_{bs} (V)"
    "v_{cs} (V)"
};

abcVoltageFiles = {
    "comparacion_v_a.pdf"
    "comparacion_v_b.pdf"
    "comparacion_v_c.pdf"
};

for k = 1:3
    exportTimeComparison(plotter, tVabcNL, ...
        vabcNL(:, k), vabcLTIonNL(:, k), ...
        abcVoltageTitles{k}, abcVoltageYLabels{k}, abcVoltageFiles{k}, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport);
end

%% 7. Curva paramétrica torque-velocidad

% Se alinea el torque con el tiempo de la velocidad de cada modelo antes de
% construir la curva paramétrica.
TmNLonWTime  = alignSignal(tTmNL,  TmNL,  tWNL);
TmLTIonWTime = alignSignal(tTmLTI, TmLTI, tWLTI);

exportXYComparison(plotter, ...
    wNL, TmNLonWTime, ...
    wLTI, TmLTIonWTime, ...
    "Curva paramétrica torque-velocidad", ...
    "\omega_m (rad/s)", ...
    "T_m (N·m)", ...
    "torque_vs_velocidad.pdf", ...
    true, closeAfterExport);

%% 8. Plano de corrientes i_d - i_q

% Eje horizontal: i_d. Eje vertical: i_q.
exportXYComparison(plotter, ...
    iqd0NL(:, 2),  iqd0NL(:, 1), ...
    iqd0LTI(:, 2), iqd0LTI(:, 1), ...
    "Trayectoria de la corriente en el plano d-q", ...
    "i_{ds}^{r} (A)", ...
    "i_{qs}^{r} (A)", ...
    "plano_corrientes_dq.pdf", ...
    true, closeAfterExport);

%% Finalización

fprintf('\nGráficos generados correctamente.\n');
fprintf('Carpeta de salida:\n%s\n\n', exportFolder);


%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

function signal = getRequiredOutput(out, signalName)
%GETREQUIREDOUTPUT Obtiene una salida obligatoria de SimulationOutput.

    [found, signal] = tryGetOutput(out, signalName);

    if ~found
        error('No se encontró la señal obligatoria out.%s.', signalName);
    end
end


function [found, signal] = tryGetOutput(out, signalName)
%TRYGETOUTPUT Obtiene una señal de struct o Simulink.SimulationOutput.

    found = false;
    signal = [];

    % Acceso dinámico mediante punto.
    try
        signal = out.(signalName);
        found = ~isempty(signal);
        if found
            return;
        end
    catch
    end

    % Método get de Simulink.SimulationOutput.
    try
        signal = out.get(signalName);
        found = ~isempty(signal);
    catch
        found = false;
        signal = [];
    end
end


function [t, y] = getSimSignal(signal)
%GETSIMSIGNAL Extrae tiempo y datos de formatos habituales de Simulink:
%   - timeseries
%   - Simulink.Timeseries
%   - Structure With Time
%   - timetable
%   - DatasetElement con propiedad Values
%
% La salida y queda siempre organizada como:
%   número de muestras x número de componentes.

    % DatasetElement
    if isobject(signal) && isprop(signal, 'Values')
        signal = signal.Values;
    end

    if isa(signal, 'timeseries')
        t = signal.Time;
        y = signal.Data;

    elseif isstruct(signal) && ...
            isfield(signal, 'time') && ...
            isfield(signal, 'signals')

        t = signal.time;
        y = signal.signals.values;

    elseif istimetable(signal)
        t = seconds(signal.Properties.RowTimes ...
                    - signal.Properties.RowTimes(1));
        y = signal.Variables;

    else
        error(['Formato de señal no reconocido. Configura los bloques ', ...
               'To Workspace como Timeseries o Structure With Time.']);
    end

    t = double(t(:));
    y = double(y);
    y = orientSamplesByRows(y, numel(t));

    if any(~isfinite(t))
        error('El vector temporal contiene NaN o Inf.');
    end
end


function y = orientSamplesByRows(y, numberOfSamples)
%ORIENTSAMPLESBYROWS Coloca la dimensión temporal en las filas.

    if isempty(y)
        error('La señal no contiene datos.');
    end

    if numel(y) == numberOfSamples
        y = reshape(y, numberOfSamples, 1);
        return;
    end

    dimensions = size(y);
    timeDimension = find(dimensions == numberOfSamples, 1, 'first');

    if isempty(timeDimension)
        error(['No se pudo identificar la dimensión temporal. ', ...
               'Se esperaban %d muestras y el tamaño recibido fue [%s].'], ...
               numberOfSamples, num2str(dimensions));
    end

    permutation = [timeDimension, ...
                   setdiff(1:ndims(y), timeDimension, 'stable')];

    y = permute(y, permutation);
    y = reshape(y, numberOfSamples, []);
end


function requireComponents(y, expectedComponents, signalName)
%REQUIRECOMPONENTS Verifica la cantidad de columnas de una señal vectorial.

    actualComponents = size(y, 2);

    if actualComponents ~= expectedComponents
        error(['La señal %s debe contener %d componentes, ', ...
               'pero se encontraron %d. Revisa el bloque Mux o ', ...
               'Vector Concatenate en Simulink.'], ...
               signalName, expectedComponents, actualComponents);
    end
end


function yAligned = alignSignal(tOriginal, yOriginal, tReference)
%ALIGNSIGNAL Interpola una señal sobre un vector temporal de referencia.

    tOriginal  = tOriginal(:);
    tReference = tReference(:);

    if numel(tOriginal) == numel(tReference)
        tolerance = 100 * eps(max(1, max(abs(tReference))));
        sameTime = all(abs(tOriginal - tReference) <= tolerance);
    else
        sameTime = false;
    end

    if sameTime
        yAligned = yOriginal;
    else
        yAligned = interp1(tOriginal, yOriginal, tReference, ...
                           'linear', 'extrap');
    end
end


function exportTimeComparison(plotter, t, yNL, yLTI, ...
        plotTitle, yLabelText, fileName, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport)
%EXPORTTIMECOMPARISON Grafica una misma variable para NL y LTI.

    [fig, ax] = plotter.plotTime( ...
        t, ...
        [yNL, yLTI], ...
        Title   = plotTitle, ...
        XLabel  = "Tiempo (s)", ...
        YLabel  = yLabelText, ...
        Legends = {"Modelo no lineal", "Modelo LTI"});

    ax = resolveAxes(fig, ax);
    formatTimeAxis(ax, t, dtMajor, dtMinor);

    if showChangeLines
        addChangeLines(ax, changeTimes);
    end

    plotter.export(fig, fileName);
    closeIfRequested(fig, closeAfterExport);
end


function exportVectorTimePlot(plotter, t, y, ...
        plotTitle, yLabelText, legendTexts, fileName, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport)
%EXPORTVECTORTIMEPLOT Grafica conjuntamente las tres componentes.

    [fig, ax] = plotter.plotTime( ...
        t, ...
        y, ...
        Title   = plotTitle, ...
        XLabel  = "Tiempo (s)", ...
        YLabel  = yLabelText, ...
        Legends = legendTexts);

    ax = resolveAxes(fig, ax);
    formatTimeAxis(ax, t, dtMajor, dtMinor);

    if showChangeLines
        addChangeLines(ax, changeTimes);
    end

    plotter.export(fig, fileName);
    closeIfRequested(fig, closeAfterExport);
end


function exportInputPlot(plotter, t, y, ...
        plotTitle, yLabelText, legendTexts, fileName, ...
        dtMajor, dtMinor, showChangeLines, changeTimes, closeAfterExport)
%EXPORTINPUTPLOT Grafica una entrada por tramos mediante stairs si existe.

    if ismethod(plotter, 'plotTimeStairs')
        [fig, ax] = plotter.plotTimeStairs( ...
            t, ...
            y, ...
            Title   = plotTitle, ...
            XLabel  = "Tiempo (s)", ...
            YLabel  = yLabelText, ...
            Legends = legendTexts);
    else
        warning(['SignalPlotter no contiene plotTimeStairs. ', ...
                 'La entrada se graficará mediante plotTime.']);

        [fig, ax] = plotter.plotTime( ...
            t, ...
            y, ...
            Title   = plotTitle, ...
            XLabel  = "Tiempo (s)", ...
            YLabel  = yLabelText, ...
            Legends = legendTexts);
    end

    ax = resolveAxes(fig, ax);
    formatTimeAxis(ax, t, dtMajor, dtMinor);

    if showChangeLines
        addChangeLines(ax, changeTimes);
    end

    plotter.export(fig, fileName);
    closeIfRequested(fig, closeAfterExport);
end


function exportXYComparison(plotter, xNL, yNL, xLTI, yLTI, ...
        plotTitle, xLabelText, yLabelText, fileName, ...
        drawZeroAxes, closeAfterExport)
%EXPORTXYCOMPARISON Grafica dos trayectorias paramétricas NL y LTI.

    fig = figure('Color', 'w');
    ax = axes(fig);

    hold(ax, 'on');
    plot(ax, xNL,  yNL,  'LineWidth', 1.25, ...
         'DisplayName', 'Modelo no lineal');
    plot(ax, xLTI, yLTI, '--', 'LineWidth', 1.25, ...
         'DisplayName', 'Modelo LTI');

    if drawZeroAxes
        if exist('xline', 'file') == 2
            xline(ax, 0, ':', 'HandleVisibility', 'off');
            yline(ax, 0, ':', 'HandleVisibility', 'off');
        else
            xl = xlim(ax);
            yl = ylim(ax);
            line(ax, [0 0], yl, 'LineStyle', ':', ...
                 'HandleVisibility', 'off');
            line(ax, xl, [0 0], 'LineStyle', ':', ...
                 'HandleVisibility', 'off');
        end
    end

    title(ax, plotTitle);
    xlabel(ax, xLabelText);
    ylabel(ax, yLabelText);
    grid(ax, 'on');
    ax.XMinorGrid = 'on';
    ax.YMinorGrid = 'on';
    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    legend(ax, 'Location', 'best');
    box(ax, 'on');

    plotter.export(fig, fileName);
    closeIfRequested(fig, closeAfterExport);
end


function ax = resolveAxes(fig, ax)
%RESOLVEAXES Devuelve un eje válido aunque SignalPlotter no lo retorne.

    if nargin < 2 || isempty(ax) || ~isgraphics(ax, 'axes')
        axesList = findall(fig, 'Type', 'axes');

        if isempty(axesList)
            error('La figura generada no contiene ejes.');
        end

        ax = axesList(1);
    end
end


function formatTimeAxis(ax, t, dtMajor, dtMinor)
%FORMATTIMEAXIS Configura el eje temporal y las grillas.

    t = t(:);

    tMin = floor(min(t) / dtMajor) * dtMajor;
    tMax = ceil(max(t)  / dtMajor) * dtMajor;

    if tMax <= tMin
        tMax = tMin + dtMajor;
    end

    xlim(ax, [tMin tMax]);
    xticks(ax, tMin:dtMajor:tMax);

    grid(ax, 'on');
    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    ax.XMinorGrid = 'on';
    ax.YMinorGrid = 'on';

    if isprop(ax, 'XAxis') && isprop(ax.XAxis, 'MinorTickValues')
        ax.XAxis.MinorTickValues = tMin:dtMinor:tMax;
    end

    if isprop(ax, 'XAxis') && isprop(ax.XAxis, 'Exponent')
        ax.XAxis.Exponent = 0;
    end
end


function changeTimes = detectChangeTimes(t, y)
%DETECTCHANGETIMES Detecta los instantes de cambio de una señal por tramos.

    t = t(:);
    dy = diff(y, 1, 1);

    scale = max(1, max(abs(y), [], 'all'));
    tolerance = 1e-10 * scale;

    if isvector(dy)
        indexes = find(abs(dy) > tolerance);
    else
        indexes = find(any(abs(dy) > tolerance, 2));
    end

    changeTimes = t(indexes + 1);
    changeTimes = changeTimes(changeTimes > min(t));
end


function addChangeLines(ax, changeTimes)
%ADDCHANGELINES Agrega líneas verticales en los cambios de entrada.

    if isempty(changeTimes)
        return;
    end

    holdState = ishold(ax);
    hold(ax, 'on');
    yLimits = ylim(ax);

    for k = 1:numel(changeTimes)
        tk = changeTimes(k);

        if exist('xline', 'file') == 2
            xline(ax, tk, '--', ...
                'HandleVisibility', 'off', ...
                'LineWidth', 0.8);
        else
            line(ax, [tk tk], yLimits, ...
                'LineStyle', '--', ...
                'LineWidth', 0.8, ...
                'HandleVisibility', 'off');
        end
    end

    ylim(ax, yLimits);

    if ~holdState
        hold(ax, 'off');
    end
end


function closeIfRequested(fig, closeAfterExport)
%CLOSEIFREQUESTED Cierra una figura luego de exportarla.

    if closeAfterExport && isgraphics(fig)
        close(fig);
    end
end
