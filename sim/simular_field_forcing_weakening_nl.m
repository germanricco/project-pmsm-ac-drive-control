%% Field-forcing y field-weakening: análisis del modelo no lineal
% -------------------------------------------------------------------------
% Antes de ejecutar este script:
%   1) Ejecute el script de parámetros de la planta.
%   2) Abra el modelo de Simulink.
%   3) Configure el bloque Step adicional del eje d con:
%          Step time     = t_step_vd
%          Initial value = 0
%          Final value   = Delta_vd
%          Sample time   = 0
%   4) Verifique que el modelo registre:
%          out.w_m_nl
%          out.T_m_nl
%          out.i_qd0_nl
%      con el orden qd0 = [q, d, 0].
%   5) SignalPlotter.m debe estar en la carpeta ../utils.
%
% El script NO usa clear ni clear all.
%
% Se ejecutan automáticamente tres casos:
%      Delta_vd = +1.9596 V
%      Delta_vd = -1.9596 V
%      Delta_vd =  0 V
%
% Se procesan y grafican únicamente las señales del modelo no lineal.
% -------------------------------------------------------------------------

close all;
clc;

%% Configuración de la consigna adicional en el eje d

t_step_vd = 0.5;                 % [s]
Delta_vd_nom = 1.9596;           % [V] = Vqs_nom/10

casos(1).nombre = 'vd_positivo';
casos(1).Delta_vd = +Delta_vd_nom;
casos(1).leyenda = 'v_{ds}^{r*}=+1.9596 V';

casos(2).nombre = 'vd_negativo';
casos(2).Delta_vd = -Delta_vd_nom;
casos(2).leyenda = 'v_{ds}^{r*}=-1.9596 V';

casos(3).nombre = 'vd_cero';
casos(3).Delta_vd = 0;
casos(3).leyenda = 'v_{ds}^{r*}=0 V';

%% Configuración de las figuras

% Ventana principal: muestra el transitorio completo alrededor del escalón.
timeWindow = [0.48, 0.60];       % [s]
dtMajor = 0.01;                  % marcas principales [s]
dtMinor = 0.005;                 % marcas menores [s]

% Figura adicional para mostrar claramente la separación de velocidad
% en régimen estacionario.
generateSteadyStateDetail = true;
steadyStateWindow = [0.54, 0.60]; % [s]
dtMajorDetail = 0.01;
dtMinorDetail = 0.005;

% Detalle del torque alrededor de los mínimos, donde la separación entre
% los tres casos es máxima. No conviene ampliar el régimen final porque
% allí los torques vuelven a ser prácticamente iguales.
generateTorqueDetail = true;
torqueDetailWindow = [0.513, 0.530]; % [s]
dtMajorTorqueDetail = 0.002;
dtMinorTorqueDetail = 0.001;

% Figura adicional de incremento de torque respecto del caso base:
%   Delta Tm = Tm(Delta_vd) - Tm(Delta_vd = 0)
% Esta gráfica permite cuantificar diferencias pequeñas sin alterar las
% curvas originales.
generateTorqueDifference = true;
torqueDifferenceWindow = [0.500, 0.545]; % [s]
dtMajorTorqueDifference = 0.005;
dtMinorTorqueDifference = 0.001;

closeAfterExport = true;

% La escala vertical se obtiene exclusivamente de los datos visibles.
% Esto evita que SignalPlotter fuerce la inclusión de y = 0 y produzca
% límites como [-600, 600] rad/s para una señal situada cerca de 415 rad/s.
verticalPaddingFraction = 0.08;

% Márgenes mínimos para evitar escalas excesivamente cerradas.
minimumPaddingSpeedMain       = 1.0;    % [rad/s]
minimumPaddingSpeedDetail     = 0.25;   % [rad/s]
minimumPaddingTorque          = 0.01;   % [N·m], figura principal
minimumPaddingTorqueDetail    = 0.001;  % [N·m], detalle del mínimo
minimumPaddingTorqueDifference = 2e-4; % [N·m], diferencia respecto al caso base
minimumPaddingCurrent         = 0.10;   % [A]

% Variables utilizadas por el Step. No se modifican los demás parámetros.
Delta_vd = 0;

%% Detectar el modelo de Simulink abierto

currentSystem = gcs;

if isempty(currentSystem)
    error(['No hay un modelo de Simulink activo. Abra el modelo, ', ...
           'seleccione alguna zona y vuelva a ejecutar el script.']);
end

modelName = bdroot(currentSystem);

if isempty(modelName) || strcmpi(modelName, 'Simulink')
    error(['No se pudo identificar el modelo. Seleccione alguna zona del ', ...
           'modelo de Simulink y vuelva a ejecutar el script.']);
end

load_system(modelName);
fprintf('Modelo utilizado: %s\n', modelName);

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

exportFolder = fullfile(scriptFolder, '..', 'docs', 'img', ...
                        'field_forcing_weakening');

if ~isfolder(exportFolder)
    mkdir(exportFolder);
end

plotter = SignalPlotter(exportFolder);

%% Ejecutar las tres simulaciones

simulationOutputs = cell(1, numel(casos));

for k = 1:numel(casos)

    Delta_vd = casos(k).Delta_vd;

    fprintf('\nSimulando caso %d/%d: Delta_vd = %+.4f V...\n', ...
        k, numel(casos), Delta_vd);

    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setVariable('t_step_vd', t_step_vd);
    simIn = simIn.setVariable('Delta_vd', Delta_vd);

    % Se respetan StartTime y StopTime configurados en el modelo.
    simulationOutputs{k} = sim(simIn);
end

% Dejar el modelo nuevamente en el caso base.
Delta_vd = 0;

%% Guardar las respuestas completas

resultsFile = fullfile(scriptFolder, ...
    'resultados_field_forcing_weakening_nl.mat');

save(resultsFile, ...
    'simulationOutputs', 'casos', ...
    't_step_vd', 'Delta_vd_nom', '-v7.3');

fprintf('\nResultados completos guardados en:\n%s\n', resultsFile);

%% Extraer las señales del modelo no lineal

legendTexts = {casos.leyenda};

% Velocidad angular
[tWNL, wNL] = collectCases(simulationOutputs, 'w_m_nl');

% Torque electromagnético
[tTmNL, TmNL] = collectCases(simulationOutputs, 'T_m_nl');

% Corriente directa: qd0 = [q, d, 0], por lo tanto i_d es componente 2.
[tIdNL, idNL] = collectCases(simulationOutputs, 'i_qd0_nl', 2);

%% Exportar figuras principales con escala vertical automática

exportThreeCasePlot(plotter, tWNL, wNL, ...
    '', ...
    '\omega_m (rad/s)', ...
    legendTexts, ...
    'velocidad_vd_tres_casos_nl.pdf', ...
    timeWindow, dtMajor, dtMinor, t_step_vd, ...
    verticalPaddingFraction, minimumPaddingSpeedMain, ...
    closeAfterExport);

exportThreeCasePlot(plotter, tTmNL, TmNL, ...
    '', ...
    'T_m (N·m)', ...
    legendTexts, ...
    'torque_vd_tres_casos_nl.pdf', ...
    timeWindow, dtMajor, dtMinor, t_step_vd, ...
    verticalPaddingFraction, minimumPaddingTorque, ...
    closeAfterExport);

exportThreeCasePlot(plotter, tIdNL, idNL, ...
    '', ...
    'i_{ds}^{r} (A)', ...
    legendTexts, ...
    'corriente_id_vd_tres_casos_nl.pdf', ...
    timeWindow, dtMajor, dtMinor, t_step_vd, ...
    verticalPaddingFraction, minimumPaddingCurrent, ...
    closeAfterExport);

%% Figura adicional: detalle de velocidad en régimen

if generateSteadyStateDetail
    exportThreeCasePlot(plotter, tWNL, wNL, ...
        '', ...
        '\omega_m (rad/s)', ...
        legendTexts, ...
        'velocidad_vd_tres_casos_nl_detalle.pdf', ...
        steadyStateWindow, dtMajorDetail, dtMinorDetail, t_step_vd, ...
        verticalPaddingFraction, minimumPaddingSpeedDetail, ...
        closeAfterExport);
end

%% Figura adicional: detalle del torque en el entorno de los mínimos

if generateTorqueDetail
    exportThreeCasePlot(plotter, tTmNL, TmNL, ...
        '', ...
        'T_m (N·m)', ...
        legendTexts, ...
        'torque_vd_tres_casos_nl_detalle.pdf', ...
        torqueDetailWindow, ...
        dtMajorTorqueDetail, dtMinorTorqueDetail, t_step_vd, ...
        verticalPaddingFraction, minimumPaddingTorqueDetail, ...
        closeAfterExport);
end

%% Figura adicional: diferencia de torque respecto del caso base

if generateTorqueDifference
    deltaTm = [ ...
        TmNL(:, 1) - TmNL(:, 3), ...
        TmNL(:, 2) - TmNL(:, 3)];

    torqueDifferenceLegends = { ...
        'T_m(+1.9596\,V)-T_m(0\,V)', ...
        'T_m(-1.9596\,V)-T_m(0\,V)'};

    exportThreeCasePlot(plotter, tTmNL, deltaTm, ...
        '', ...
        '\Delta T_m (N·m)', ...
        torqueDifferenceLegends, ...
        'diferencia_torque_respecto_base_nl.pdf', ...
        torqueDifferenceWindow, ...
        dtMajorTorqueDifference, dtMinorTorqueDifference, t_step_vd, ...
        verticalPaddingFraction, minimumPaddingTorqueDifference, ...
        closeAfterExport);
end

%% Resumen numérico de las diferencias

printCaseSummary(tWNL, wNL, tTmNL, TmNL, tIdNL, idNL, ...
                 t_step_vd, casos);

%% Finalización

fprintf('\nGráficos generados correctamente.\n');
fprintf('Carpeta de salida:\n%s\n\n', exportFolder);


%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

function [tReference, dataMatrix] = collectCases( ...
        simulationOutputs, signalName, componentIndex)
%COLLECTCASES Extrae una misma señal para todas las simulaciones.
%
% La salida dataMatrix contiene una columna por caso.

    if nargin < 3
        componentIndex = [];
    end

    numberOfCases = numel(simulationOutputs);
    tReference = [];
    dataMatrix = [];

    for k = 1:numberOfCases

        signal = getRequiredOutput(simulationOutputs{k}, signalName);
        [tCase, yCase] = getSimSignal(signal);

        if isempty(componentIndex)
            if size(yCase, 2) ~= 1
                error(['La señal out.%s no es escalar. Indique la ', ...
                       'componente que desea extraer.'], signalName);
            end

            yCase = yCase(:, 1);

        else
            if size(yCase, 2) < componentIndex
                error(['La señal out.%s no contiene la componente %d. ', ...
                       'Se encontraron %d componentes.'], ...
                       signalName, componentIndex, size(yCase, 2));
            end

            yCase = yCase(:, componentIndex);
        end

        if k == 1
            tReference = tCase;
            dataMatrix = zeros(numel(tReference), numberOfCases);
            dataMatrix(:, k) = yCase;
        else
            dataMatrix(:, k) = alignSignal(tCase, yCase, tReference);
        end
    end
end


function signal = getRequiredOutput(out, signalName)
%GETREQUIREDOUTPUT Obtiene una salida obligatoria de SimulationOutput.

    [found, signal] = tryGetOutput(out, signalName);

    if ~found
        error(['No se encontró la señal obligatoria out.%s. ', ...
               'Revise el nombre del bloque To Workspace.'], signalName);
    end
end


function [found, signal] = tryGetOutput(out, signalName)
%TRYGETOUTPUT Obtiene una señal de struct o Simulink.SimulationOutput.

    found = false;
    signal = [];

    try
        signal = out.(signalName);
        found = ~isempty(signal);
        if found
            return;
        end
    catch
    end

    try
        signal = out.get(signalName);
        found = ~isempty(signal);
    catch
        found = false;
        signal = [];
    end
end


function [t, y] = getSimSignal(signal)
%GETSIMSIGNAL Extrae tiempo y datos de formatos habituales de Simulink.

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
        error(['Formato de señal no reconocido. Configure los bloques ', ...
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
               'Se esperaban %d muestras y se recibió el tamaño [%s].'], ...
               numberOfSamples, num2str(dimensions));
    end

    permutation = [timeDimension, ...
                   setdiff(1:ndims(y), timeDimension, 'stable')];

    y = permute(y, permutation);
    y = reshape(y, numberOfSamples, []);
end


function yAligned = alignSignal(tOriginal, yOriginal, tReference)
%ALIGNSIGNAL Interpola una señal sobre el vector temporal de referencia.

    tOriginal = tOriginal(:);
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


function exportThreeCasePlot(plotter, t, y, ...
        plotTitle, yLabelText, legendTexts, fileName, ...
        timeWindow, dtMajor, dtMinor, stepTime, ...
        verticalPaddingFraction, minimumPadding, closeAfterExport)
%EXPORTTHREECASEPLOT Grafica los tres valores de Delta_vd.
%
% Los límites verticales se calculan sólo con los datos contenidos en
% timeWindow. De esta forma no se obliga al eje a incluir el origen.

    [fig, ax] = plotter.plotTime( ...
        t, ...
        y, ...
        Title   = plotTitle, ...
        XLabel  = 'Tiempo (s)', ...
        YLabel  = yLabelText, ...
        Legends = legendTexts);

    ax = resolveAxes(fig, ax);

    [tMin, tMax] = formatTimeAxis( ...
        ax, t, timeWindow, dtMajor, dtMinor);

    autoScaleYAxis(ax, t, y, [tMin, tMax], ...
                   verticalPaddingFraction, minimumPadding);

    if stepTime >= tMin && stepTime <= tMax && ...
            exist('xline', 'file') == 2
        xline(ax, stepTime, '--', ...
            'HandleVisibility', 'off', ...
            'LineWidth', 0.8);
    end

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


function [tMin, tMax] = formatTimeAxis( ...
        ax, t, timeWindow, dtMajor, dtMinor)
%FORMATTIMEAXIS Configura el intervalo temporal y las grillas.

    t = t(:);

    requestedMin = timeWindow(1);
    requestedMax = timeWindow(2);

    tMin = max(min(t), requestedMin);
    tMax = min(max(t), requestedMax);

    if tMax <= tMin
        error(['La ventana temporal [%g, %g] s no está contenida en ', ...
               'el intervalo simulado [%g, %g] s.'], ...
               requestedMin, requestedMax, min(t), max(t));
    end

    xlim(ax, [tMin tMax]);
    xticks(ax, tMin:dtMajor:tMax);

    grid(ax, 'on');
    box(ax, 'on');

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


function autoScaleYAxis(ax, t, y, timeWindow, ...
        paddingFraction, minimumPadding)
%AUTOSCALEYAXIS Ajusta el eje y con los datos visibles.
%
% No recorta datos ni altera las curvas; sólo define límites adecuados
% para el intervalo temporal mostrado.

    mask = t >= timeWindow(1) & t <= timeWindow(2);

    if ~any(mask)
        error('No hay muestras dentro de la ventana temporal solicitada.');
    end

    visibleData = y(mask, :);
    visibleData = visibleData(isfinite(visibleData));

    if isempty(visibleData)
        error('No hay datos finitos en la ventana temporal solicitada.');
    end

    yMin = min(visibleData);
    yMax = max(visibleData);
    dataSpan = yMax - yMin;

    padding = max(minimumPadding, paddingFraction * dataSpan);

    if dataSpan <= 100 * eps(max(1, max(abs(visibleData))))
        center = 0.5 * (yMin + yMax);
        ylim(ax, [center - padding, center + padding]);
    else
        ylim(ax, [yMin - padding, yMax + padding]);
    end
end


function printCaseSummary(tW, w, tTm, Tm, tId, id, stepTime, casos)
%PRINTCASESUMMARY Muestra las diferencias reales entre los tres casos.

    maskW = tW >= stepTime;
    maskTm = tTm >= stepTime;
    maskId = tId >= stepTime;

    fprintf('\n====================================================\n');
    fprintf('Resumen numérico del modelo no lineal\n');
    fprintf('====================================================\n');

    fprintf('Máx. |omega(+vd) - omega(0)| = %.9g rad/s\n', ...
        max(abs(w(maskW, 1) - w(maskW, 3))));

    fprintf('Máx. |omega(-vd) - omega(0)| = %.9g rad/s\n', ...
        max(abs(w(maskW, 2) - w(maskW, 3))));

    fprintf('Máx. |Tm(+vd) - Tm(0)|       = %.9g N·m\n', ...
        max(abs(Tm(maskTm, 1) - Tm(maskTm, 3))));

    fprintf('Máx. |Tm(-vd) - Tm(0)|       = %.9g N·m\n', ...
        max(abs(Tm(maskTm, 2) - Tm(maskTm, 3))));

    fprintf('Máx. |id(+vd) - id(0)|       = %.9g A\n', ...
        max(abs(id(maskId, 1) - id(maskId, 3))));

    fprintf('Máx. |id(-vd) - id(0)|       = %.9g A\n', ...
        max(abs(id(maskId, 2) - id(maskId, 3))));

    % Promedio del último 10 % de las muestras para estimar régimen.
    n = size(w, 1);
    finalStart = max(1, floor(0.9 * n));

    fprintf('\nVelocidad media en el tramo final:\n');
    for k = 1:numel(casos)
        fprintf('  %-12s: %.9g rad/s\n', ...
            casos(k).nombre, mean(w(finalStart:end, k)));
    end

    fprintf('====================================================\n\n');
end


function closeIfRequested(fig, closeAfterExport)
%CLOSEIFREQUESTED Cierra la figura luego de exportarla.

    if closeAfterExport && isgraphics(fig)
        close(fig);
    end
end
