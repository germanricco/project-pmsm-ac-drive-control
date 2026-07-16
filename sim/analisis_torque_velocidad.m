%% Curva paramétrica torque-velocidad: análisis y comparación NL-LTI
% -------------------------------------------------------------------------
% Ejecutar DESPUÉS de simular el modelo de Simulink y obtener la variable
% "out" en el workspace.
%
% Señales requeridas en out:
%   w_m_nl, T_m_nl
%   w_m_lti, T_m_lti
%   v_qs_sim
%
% Señales opcionales:
%   T_ld_sim, T_s_nl
%
% El script genera:
%   1) torque_velocidad_nl_analisis.pdf
%      Trayectoria completa del modelo NL, cuadrantes, flechas, puntos de
%      conmutación y características cuasi-estacionarias.
%
%   2) torque_velocidad_nl_vs_lti.pdf
%      Comparación limpia entre los modelos NL y LTI.
%
%   3) torque_velocidad_por_polaridad.pdf (opcional)
%      Separación auxiliar de los tramos con v_qs > 0 y v_qs < 0.
%
% También exporta:
%   puntos_torque_velocidad.csv
%
% No se usa clear ni clear all para conservar los parámetros del sistema.
% -------------------------------------------------------------------------

close all;
clc;

%% 1. Configuración

exportPDF = true;
closeAfterExport = false;
generateSplitFigure = true;

showQuasiStationary = true;
showDirectionArrows = true;
showQuadrantLabels = true;
showPointLabels = true;

% Para evitar saturar la figura, por defecto se etiquetan solamente el
% inicio, los cambios de v_qs y el punto final. Todos los cambios de entrada
% siguen apareciendo como marcadores y quedan registrados en el archivo CSV.
% Valores posibles: "representative", "all" o "none".
pointLabelMode = "representative";

numberOfArrows = 7;
lineWidthNL = 1.8;
lineWidthLTI = 1.5;
lineWidthQS = 1.0;

% Parámetros de la PMSM especificados por la guía del proyecto.
% Se usan nombres propios del script para no sobrescribir variables del
% modelo ni depender de nombres inexistentes en el workspace.
Pp_QS = 3;                 % pares de polos
lambda_pm_QS = 0.016;      % flujo de imanes concatenado [Wb-turn]

% Parámetros térmicos. Se leen del workspace si existen; de lo contrario,
% se usan los valores especificados por la guía.
Rs_REF_QS = readBaseScalar('Rs_REF', 1.02);   % [ohm] a Ts_REF
alpha_Cu_QS = readBaseScalar('alpha_Cu', 0.0039);
Ts_REF_QS = readBaseScalar('Ts_REF', 20);     % [°C]

% true: calcula Rs para las curvas QS a la temperatura media del modelo NL.
% false: usa directamente Rs_REF_QS, es decir, la resistencia a Ts_REF.
useMeanNLTemperatureForQS = true;

%% 2. Rutas de exportación

scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder)
    scriptFolder = pwd;
end

exportFolder = fullfile(scriptFolder, '..', 'docs', 'img', ...
                        'simulacion_nl_lti');

if ~isfolder(exportFolder)
    mkdir(exportFolder);
end

%% 3. Validar SimulationOutput

if ~exist('out', 'var')
    error(['No existe la variable "out" en el workspace. ', ...
           'Ejecuta primero el modelo de Simulink.']);
end

%% 4. Extraer y alinear señales

[tWNL, wNL] = getSimSignal(getRequiredOutput(out, 'w_m_nl'));
[tTmNL, TmNL] = getSimSignal(getRequiredOutput(out, 'T_m_nl'));

[tWLTI, wLTI] = getSimSignal(getRequiredOutput(out, 'w_m_lti'));
[tTmLTI, TmLTI] = getSimSignal(getRequiredOutput(out, 'T_m_lti'));

TmNLonWTime = alignSignal(tTmNL, TmNL, tWNL, 'linear');
TmLTIonWTime = alignSignal(tTmLTI, TmLTI, tWLTI, 'linear');

[hasVqs, vqsSignal] = tryGetOutput(out, 'v_qs_sim');
if ~hasVqs
    error(['No se encontró out.v_qs_sim. Esta señal es necesaria para ', ...
           'identificar los cambios y construir las curvas QS.']);
end
[tVqs, vqs] = getSimSignal(vqsSignal);

[hasTld, TldSignal] = tryGetOutput(out, 'T_ld_sim');
if hasTld
    [tTld, Tld] = getSimSignal(TldSignal);
else
    tTld = [];
    Tld = [];
    warning('No se encontró out.T_ld_sim. La tabla se generará sin T_ld.');
end

[hasTsNL, TsSignal] = tryGetOutput(out, 'T_s_nl');
if hasTsNL
    [~, TsNL] = getSimSignal(TsSignal);
else
    TsNL = [];
end

% Entrada v_qs alineada con la trayectoria NL. Se usa interpolación tipo
% "previous" porque la entrada está definida por tramos constantes.
vqsOnNL = alignSignal(tVqs, vqs, tWNL, 'previous');

if hasTld
    TldOnNL = alignSignal(tTld, Tld, tWNL, 'previous');
else
    TldOnNL = nan(size(tWNL));
end

%% 5. Resistencia congelada para las curvas cuasi-estacionarias

if useMeanNLTemperatureForQS && hasTsNL && ~isempty(TsNL)
    Ts_QS = mean(TsNL, 'omitnan');
    Rs_QS = Rs_REF_QS * (1 + alpha_Cu_QS * (Ts_QS - Ts_REF_QS));
    qsResistanceText = sprintf('R_s = %.3f \\Omega, T_s \\approx %.1f ^\\circC', ...
                               Rs_QS, Ts_QS);
else
    Ts_QS = Ts_REF_QS;
    Rs_QS = Rs_REF_QS;
    qsResistanceText = sprintf('R_s = R_{s,REF} = %.3f \\Omega', Rs_QS);
end

fprintf('\nParámetros usados en las curvas cuasi-estacionarias:\n');
fprintf('Pp       = %.0f pares de polos\n', Pp_QS);
fprintf('lambda_m = %.6f Wb-turn\n', lambda_pm_QS);
fprintf('Rs_QS    = %.6f ohm\n', Rs_QS);
fprintf('Ts_QS    = %.3f °C\n\n', Ts_QS);

%% 6. Detectar instantes de cambio y preparar tabla de puntos

changeTimesVqs = detectChangeTimes(tVqs, vqs);

if hasTld
    changeTimesTld = detectChangeTimes(tTld, Tld);
else
    changeTimesTld = [];
end

changeTimes = unique([changeTimesVqs(:); changeTimesTld(:)]);
changeTimes = changeTimes(changeTimes > tWNL(1) & changeTimes < tWNL(end));

% Se incluyen el inicio, todos los cambios y el final.
markTimes = unique([tWNL(1); changeTimes; tWNL(end)]);
idxMarkNL = nearestIndices(tWNL, markTimes);

pointNames = compose("P%d", (0:numel(markTimes)-1)');
wAtPoints = wNL(idxMarkNL);
TmAtPoints = TmNLonWTime(idxMarkNL);
vqsAtPoints = samplePrevious(tVqs, vqs, markTimes);

if hasTld
    TldAtPoints = samplePrevious(tTld, Tld, markTimes);
else
    TldAtPoints = nan(size(markTimes));
end

[quadrants, operatingModes] = classifyOperatingPoint(wAtPoints, TmAtPoints);

pointTable = table(pointNames, markTimes, vqsAtPoints, TldAtPoints, ...
                   wAtPoints, TmAtPoints, quadrants, operatingModes, ...
    'VariableNames', {'Punto','Tiempo_s','v_qs_V','T_ld_Nm', ...
                      'omega_m_rad_s','T_m_Nm','Cuadrante','Modo'});

disp(pointTable);

csvPath = fullfile(exportFolder, 'puntos_torque_velocidad.csv');
writetable(pointTable, csvPath);

%% 7. Límites comunes, calculados SOLO con las trayectorias dinámicas
% Esto evita que las rectas QS compriman visualmente los bucles.

allW = [wNL(:); wLTI(:)];
allT = [TmNLonWTime(:); TmLTIonWTime(:)];

xMin = min(allW);
xMax = max(allW);
yMin = min(allT);
yMax = max(allT);

xRange = max(1, xMax - xMin);
yRange = max(1e-3, yMax - yMin);

xLimits = [xMin - 0.08*xRange, xMax + 0.08*xRange];
yLimits = [yMin - 0.12*yRange, yMax + 0.12*yRange];

wGrid = linspace(xLimits(1), xLimits(2), 800);

% Niveles de tensión de interés: máximo positivo, cero y mínimo negativo.
vqLevels = [max(vqs), 0, min(vqs)];
vqLevels = unique(round(vqLevels, 9), 'stable');
vqLevels = sort(vqLevels, 'descend');

%% 8. Figura principal: modelo NL + análisis físico + curvas QS

fig1 = figure('Color', 'w', 'Position', [80 70 1180 760]);
ax1 = axes(fig1);
hold(ax1, 'on');
box(ax1, 'on');

% Curvas cuasi-estacionarias, dibujadas primero y en gris tenue.
hQSLegend = gobjects(0);
if showQuasiStationary
    for k = 1:numel(vqLevels)
        Tqs = torqueSpeedQS(wGrid, vqLevels(k), Pp_QS, ...
                            lambda_pm_QS, Rs_QS);

        plot(ax1, wGrid, Tqs, '-.', ...
             'Color', [0.55 0.55 0.55], ...
             'LineWidth', lineWidthQS, ...
             'HandleVisibility', 'off');
    end

    hQSLegend = plot(ax1, nan, nan, '-.', ...
        'Color', [0.55 0.55 0.55], ...
        'LineWidth', lineWidthQS, ...
        'DisplayName', 'Características cuasi-estacionarias');
end

hNL = plot(ax1, wNL, TmNLonWTime, ...
    'Color', [0.0000 0.4470 0.7410], ...
    'LineWidth', lineWidthNL, ...
    'DisplayName', 'Modelo no lineal');

% Ejes de separación de cuadrantes.
xline(ax1, 0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(ax1, 0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');

xlim(ax1, xLimits);
ylim(ax1, yLimits);

% Etiquetas directas de las curvas QS dentro de la ventana visible.
if showQuasiStationary
    addQSLabels(ax1, wGrid, vqLevels, Pp_QS, lambda_pm_QS, ...
                Rs_QS, yLimits, wNL, TmNLonWTime);
end

% Flechas de sentido temporal.
if showDirectionArrows
    addDirectionArrows(ax1, wNL, TmNLonWTime, numberOfArrows, ...
                       [0.0000 0.4470 0.7410]);
end

% Puntos de conmutación numerados.
hPoints = plot(ax1, wAtPoints, TmAtPoints, 'ko', ...
    'MarkerSize', 6.5, ...
    'MarkerFaceColor', 'w', ...
    'LineWidth', 1.0, ...
    'DisplayName', 'Puntos de cambio de entrada');

if showPointLabels && pointLabelMode ~= "none"
    switch pointLabelMode
        case "all"
            labelMask = true(size(markTimes));

        case "representative"
            % Etiquetar inicio, cambios de tensión y final. Esto conserva la
            % lectura temporal sin superponer los doce rótulos.
            representativeTimes = unique([tWNL(1); changeTimesVqs(:); tWNL(end)]);
            labelMask = false(size(markTimes));
            timeTolerance = max(1e-10, 1e-7 * max(1, tWNL(end)-tWNL(1)));
            for k = 1:numel(markTimes)
                labelMask(k) = any(abs(markTimes(k)-representativeTimes) <= timeTolerance);
            end

        otherwise
            error('pointLabelMode debe ser "representative", "all" o "none".');
    end

    addPointLabelsSmart(ax1, wAtPoints(labelMask), ...
                        TmAtPoints(labelMask), pointNames(labelMask));
end

hStart = plot(ax1, wAtPoints(1), TmAtPoints(1), 's', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [0.20 0.75 0.25], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Inicio');

hEnd = plot(ax1, wAtPoints(end), TmAtPoints(end), 'd', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [0.85 0.20 0.75], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Fin');

if showQuadrantLabels
    addQuadrantLabels(ax1);
end

title(ax1, 'Trayectoria paramétrica torque-velocidad del modelo no lineal');
xlabel(ax1, '\omega_m (rad/s)');
ylabel(ax1, 'T_m (N\cdot m)');

grid(ax1, 'on');
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
ax1.XMinorTick = 'on';
ax1.YMinorTick = 'on';
ax1.FontSize = 11;

subtitle(ax1, ['Curvas QS con ', qsResistanceText]);

legendHandles = [hNL; hPoints; hStart; hEnd; hQSLegend];
legendHandles = legendHandles(isgraphics(legendHandles));
legend(ax1, legendHandles, ...
       'Location', 'southoutside', ...
       'Orientation', 'horizontal', ...
       'NumColumns', min(5, numel(legendHandles))); 

if exportPDF
    file1 = fullfile(exportFolder, 'torque_velocidad_nl_analisis.pdf');
    exportgraphics(fig1, file1, 'ContentType', 'vector');
end

%% 9. Figura de validación: comparación limpia NL vs LTI

fig2 = figure('Color', 'w', 'Position', [110 90 1080 700]);
ax2 = axes(fig2);
hold(ax2, 'on');
box(ax2, 'on');

plot(ax2, wNL, TmNLonWTime, ...
    'Color', [0.0000 0.4470 0.7410], ...
    'LineWidth', lineWidthNL, ...
    'DisplayName', 'Modelo no lineal');

plot(ax2, wLTI, TmLTIonWTime, '--', ...
    'Color', [0.8500 0.3250 0.0980], ...
    'LineWidth', lineWidthLTI, ...
    'DisplayName', 'Modelo LTI');

xline(ax2, 0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(ax2, 0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');

xlim(ax2, xLimits);
ylim(ax2, yLimits);

title(ax2, 'Comparación de trayectorias torque-velocidad');
xlabel(ax2, '\omega_m (rad/s)');
ylabel(ax2, 'T_m (N\cdot m)');

grid(ax2, 'on');
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
ax2.XMinorTick = 'on';
ax2.YMinorTick = 'on';
ax2.FontSize = 11;
legend(ax2, 'Location', 'best');

if exportPDF
    file2 = fullfile(exportFolder, 'torque_velocidad_nl_vs_lti.pdf');
    exportgraphics(fig2, file2, 'ContentType', 'vector');
end

%% 10. Figura auxiliar opcional: tramos según polaridad de v_qs

if generateSplitFigure
    vTolerance = max(1e-9, 1e-6 * max(abs(vqsOnNL)));

    positiveMask = vqsOnNL > vTolerance;
    negativeMask = vqsOnNL < -vTolerance;

    wPositive = wNL;
    tPositive = TmNLonWTime;
    wPositive(~positiveMask) = nan;
    tPositive(~positiveMask) = nan;

    wNegative = wNL;
    tNegative = TmNLonWTime;
    wNegative(~negativeMask) = nan;
    tNegative(~negativeMask) = nan;

    fig3 = figure('Color', 'w', 'Position', [70 80 1280 580]);
    layout = tiledlayout(fig3, 1, 2, 'TileSpacing', 'compact', ...
                         'Padding', 'compact');

    ax3a = nexttile(layout, 1);
    hold(ax3a, 'on');
    plot(ax3a, wPositive, tPositive, ...
        'Color', [0.0000 0.4470 0.7410], ...
        'LineWidth', lineWidthNL);
    xline(ax3a, 0, 'k-', 'HandleVisibility', 'off');
    yline(ax3a, 0, 'k-', 'HandleVisibility', 'off');
    xlim(ax3a, xLimits);
    ylim(ax3a, yLimits);
    title(ax3a, 'Tramos con v_{qs}^{r} > 0');
    xlabel(ax3a, '\omega_m (rad/s)');
    ylabel(ax3a, 'T_m (N\cdot m)');
    grid(ax3a, 'on');
    box(ax3a, 'on');

    ax3b = nexttile(layout, 2);
    hold(ax3b, 'on');
    plot(ax3b, wNegative, tNegative, ...
        'Color', [0.8500 0.3250 0.0980], ...
        'LineWidth', lineWidthNL);
    xline(ax3b, 0, 'k-', 'HandleVisibility', 'off');
    yline(ax3b, 0, 'k-', 'HandleVisibility', 'off');
    xlim(ax3b, xLimits);
    ylim(ax3b, yLimits);
    title(ax3b, 'Tramos con v_{qs}^{r} < 0');
    xlabel(ax3b, '\omega_m (rad/s)');
    ylabel(ax3b, 'T_m (N\cdot m)');
    grid(ax3b, 'on');
    box(ax3b, 'on');

    title(layout, 'Descomposición auxiliar de la trayectoria por polaridad de tensión');

    if exportPDF
        file3 = fullfile(exportFolder, 'torque_velocidad_por_polaridad.pdf');
        exportgraphics(fig3, file3, 'ContentType', 'vector');
    end
end

%% 11. Finalización

fprintf('\nFiguras generadas correctamente.\n');
fprintf('Carpeta de salida:\n%s\n', exportFolder);
fprintf('Tabla de puntos:\n%s\n\n', csvPath);

if closeAfterExport
    close(fig1);
    close(fig2);
    if generateSplitFigure && exist('fig3', 'var') && isgraphics(fig3)
        close(fig3);
    end
end

%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

function value = readBaseScalar(variableName, defaultValue)
%READBASESCALAR Lee un escalar del workspace base o usa un valor por defecto.

    existsInBase = evalin('base', sprintf('exist(''%s'',''var'')', variableName));

    if existsInBase
        candidate = evalin('base', variableName);
        if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
            value = double(candidate);
            return;
        end
    end

    value = defaultValue;
end

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

    elseif isstruct(signal) && isfield(signal, 'time') && ...
            isfield(signal, 'signals')
        t = signal.time;
        y = signal.signals.values;

    elseif istimetable(signal)
        t = seconds(signal.Properties.RowTimes - signal.Properties.RowTimes(1));
        y = signal.Variables;

    else
        error(['Formato de señal no reconocido. Configura los bloques ', ...
               'To Workspace como Timeseries o Structure With Time.']);
    end

    t = double(t(:));
    y = double(y);
    y = orientSamplesByRows(y, numel(t));

    if size(y, 2) ~= 1
        error('Se esperaba una señal escalar y se recibieron %d componentes.', ...
              size(y, 2));
    end

    y = y(:, 1);
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

function yAligned = alignSignal(tOriginal, yOriginal, tReference, method)
%ALIGNSIGNAL Interpola una señal sobre un vector temporal de referencia.

    tOriginal = tOriginal(:);
    yOriginal = yOriginal(:);
    tReference = tReference(:);

    if nargin < 4
        method = 'linear';
    end

    if numel(tOriginal) == numel(tReference)
        tolerance = 100 * eps(max(1, max(abs(tReference))));
        sameTime = all(abs(tOriginal - tReference) <= tolerance);
    else
        sameTime = false;
    end

    if sameTime
        yAligned = yOriginal;
    else
        yAligned = interp1(tOriginal, yOriginal, tReference, method, 'extrap');
    end
end

function sampledValues = samplePrevious(t, y, queryTimes)
%SAMPLEPREVIOUS Evalúa una señal por tramos en instantes determinados.

    sampledValues = interp1(t(:), y(:), queryTimes(:), 'previous', 'extrap');
end

function changeTimes = detectChangeTimes(t, y)
%DETECTCHANGETIMES Detecta cambios en una señal definida por tramos.

    t = t(:);
    y = y(:);

    dy = diff(y);
    scale = max(1, max(abs(y)));
    tolerance = 1e-9 * scale;

    indexes = find(abs(dy) > tolerance);
    changeTimes = t(indexes + 1);
end

function idx = nearestIndices(t, queryTimes)
%NEARESTINDICES Devuelve el índice más cercano a cada instante consultado.

    t = t(:);
    queryTimes = queryTimes(:);
    idx = zeros(size(queryTimes));

    for k = 1:numel(queryTimes)
        [~, idx(k)] = min(abs(t - queryTimes(k)));
    end
end

function Tqs = torqueSpeedQS(w, vq, Pp, lambdaPm, Rs)
%TORQUESPEEDQS Característica eléctrica cuasi-estacionaria para id = 0.
%
%   vq = Rs iq + Pp lambdaPm w
%   Tm = (3/2) Pp lambdaPm iq

    Kt = 1.5 * Pp * lambdaPm;
    Ke = Pp * lambdaPm;

    Tqs = (Kt/Rs) .* vq - (Kt*Ke/Rs) .* w;
end

function addQSLabels(ax, wGrid, vqLevels, Pp, lambdaPm, Rs, ...
        yLimits, wTrajectory, tTrajectory)
%ADDQSLABELS Coloca cada rótulo QS en una zona libre de la trayectoria.
%
% La posición se elige entre los puntos visibles de cada recta buscando la
% mayor separación respecto de la curva dinámica. De esta forma se evita que
% los textos tapen los puntos de conmutación o los lazos principales.

    xLimits = xlim(ax);
    xRange = diff(xLimits);
    yRange = diff(yLimits);

    trajectoryX = (wTrajectory(:) - xLimits(1)) / xRange;
    trajectoryY = (tTrajectory(:) - yLimits(1)) / yRange;

    for k = 1:numel(vqLevels)
        Tqs = torqueSpeedQS(wGrid, vqLevels(k), Pp, lambdaPm, Rs);

        visible = Tqs >= yLimits(1) & Tqs <= yLimits(2) & ...
                  wGrid >= xLimits(1) + 0.08*xRange & ...
                  wGrid <= xLimits(2) - 0.08*xRange;

        candidateIndexes = find(visible);
        if isempty(candidateIndexes)
            continue;
        end

        % Evaluar una cantidad acotada de candidatos para mantener el código
        % rápido aun cuando wGrid tenga muchas muestras.
        sampleCount = min(120, numel(candidateIndexes));
        sampled = unique(round(linspace(1, numel(candidateIndexes), sampleCount)));
        candidateIndexes = candidateIndexes(sampled);

        bestScore = -inf;
        bestIndex = candidateIndexes(1);

        for idx = candidateIndexes(:)'
            xNorm = (wGrid(idx) - xLimits(1)) / xRange;
            yNorm = (Tqs(idx) - yLimits(1)) / yRange;

            distanceToTrajectory = hypot(trajectoryX-xNorm, trajectoryY-yNorm);
            minimumDistance = min(distanceToTrajectory);

            % Penalización suave cerca de los bordes del gráfico.
            edgeDistance = min([xNorm, 1-xNorm, yNorm, 1-yNorm]);
            score = minimumDistance + 0.20*edgeDistance;

            if score > bestScore
                bestScore = score;
                bestIndex = idx;
            end
        end

        % Ángulo de la recta medido en coordenadas normalizadas de los ejes.
        Ke = Pp * lambdaPm;
        Kt = 1.5 * Pp * lambdaPm;
        slope = -(Kt*Ke/Rs);
        rotationAngle = atan2(slope*xRange, yRange) * 180/pi;

        labelText = sprintf('v_{qs}^{r}=%.2f V', vqLevels(k));

        text(ax, wGrid(bestIndex), Tqs(bestIndex), labelText, ...
            'Color', [0.32 0.32 0.32], ...
            'FontSize', 8.5, ...
            'Rotation', rotationAngle, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'BackgroundColor', 'w', ...
            'EdgeColor', [0.82 0.82 0.82], ...
            'Margin', 1.5, ...
            'Clipping', 'on');
    end
end

function addDirectionArrows(ax, x, y, numberOfArrows, arrowColor)
%ADDDIRECTIONARROWS Agrega flechas cortas en coordenadas normalizadas.
%
% No se utiliza quiver porque, al mezclar escalas muy diferentes entre
% velocidad y torque, puede producir líneas verticales de gran longitud en
% la exportación vectorial. Las flechas se construyen con una línea corta y
% un pequeño triángulo, ambos dimensionados en unidades normalizadas.

    x = x(:);
    y = y(:);

    valid = isfinite(x) & isfinite(y);
    x = x(valid);
    y = y(valid);

    if numel(x) < 10 || numberOfArrows < 1
        return;
    end

    xl = xlim(ax);
    yl = ylim(ax);
    xRange = diff(xl);
    yRange = diff(yl);

    if xRange <= 0 || yRange <= 0
        return;
    end

    xNorm = (x-xl(1))/xRange;
    yNorm = (y-yl(1))/yRange;

    arcLength = [0; cumsum(hypot(diff(xNorm), diff(yNorm)))];
    if arcLength(end) <= 0
        return;
    end

    targets = linspace(0.10, 0.90, numberOfArrows) * arcLength(end);

    shaftLength = 0.030;  % fracción aproximada del tamaño de los ejes
    headLength  = 0.011;
    headWidth   = 0.008;

    for k = 1:numel(targets)
        [~, i] = min(abs(arcLength-targets(k)));

        i0 = max(1, i-3);
        i1 = min(numel(xNorm), i+3);
        tangent = [xNorm(i1)-xNorm(i0), yNorm(i1)-yNorm(i0)];
        tangentNorm = hypot(tangent(1), tangent(2));

        if tangentNorm < 1e-8
            continue;
        end

        tangent = tangent/tangentNorm;
        normal = [-tangent(2), tangent(1)];
        center = [xNorm(i), yNorm(i)];

        pStart = center - 0.45*shaftLength*tangent;
        pTip   = center + 0.55*shaftLength*tangent;
        pBase  = pTip - headLength*tangent;

        % Mantener toda la flecha dentro de los ejes.
        allPoints = [pStart; pTip; pBase+headWidth*normal; pBase-headWidth*normal];
        if any(allPoints(:) < 0.01) || any(allPoints(:) > 0.99)
            continue;
        end

        shaftX = xl(1) + xRange*[pStart(1), pBase(1)];
        shaftY = yl(1) + yRange*[pStart(2), pBase(2)];
        line(ax, shaftX, shaftY, ...
             'Color', arrowColor, ...
             'LineWidth', 1.1, ...
             'HandleVisibility', 'off');

        headNormalized = [pTip; ...
                          pBase + headWidth*normal; ...
                          pBase - headWidth*normal];
        headX = xl(1) + xRange*headNormalized(:,1);
        headY = yl(1) + yRange*headNormalized(:,2);

        patch(ax, headX, headY, arrowColor, ...
              'EdgeColor', arrowColor, ...
              'HandleVisibility', 'off', ...
              'Clipping', 'on');
    end
end

function addPointLabelsSmart(ax, x, y, labels)
%ADDPOINTLABELSSMART Coloca rótulos con búsqueda simple de espacios libres.
%
% La comparación se realiza en coordenadas normalizadas para que la gran
% diferencia de escala entre omega y torque no distorsione las distancias.

    x = x(:);
    y = y(:);
    labels = string(labels(:));

    xl = xlim(ax);
    yl = ylim(ax);
    xRange = diff(xl);
    yRange = diff(yl);

    xNorm = (x-xl(1))/xRange;
    yNorm = (y-yl(1))/yRange;

    candidateOffsets = [
         0.024,  0.036
         0.024, -0.036
        -0.024,  0.036
        -0.024, -0.036
         0.040,  0.000
        -0.040,  0.000
         0.000,  0.055
         0.000, -0.055
         0.050,  0.045
        -0.050,  0.045
         0.050, -0.045
        -0.050, -0.045
    ];

    placedPositions = zeros(0,2);

    for k = 1:numel(x)
        bestScore = -inf;
        bestPosition = [xNorm(k)+0.024, yNorm(k)+0.036];

        for c = 1:size(candidateOffsets,1)
            candidate = [xNorm(k), yNorm(k)] + candidateOffsets(c,:);

            if any(candidate < 0.035) || any(candidate > 0.965)
                continue;
            end

            % Separación respecto de otros puntos y rótulos ya colocados.
            pointDistances = hypot(xNorm-candidate(1), yNorm-candidate(2));
            scorePoints = min(pointDistances);

            if isempty(placedPositions)
                scoreLabels = 1;
            else
                scoreLabels = min(hypot(placedPositions(:,1)-candidate(1), ...
                                        placedPositions(:,2)-candidate(2)));
            end

            score = min(scorePoints, 1.4*scoreLabels);

            if score > bestScore
                bestScore = score;
                bestPosition = candidate;
            end
        end

        xText = xl(1) + xRange*bestPosition(1);
        yText = yl(1) + yRange*bestPosition(2);

        % Línea guía corta entre el punto y el rótulo.
        line(ax, [x(k), xText], [y(k), yText], ...
             'Color', [0.35 0.35 0.35], ...
             'LineWidth', 0.6, ...
             'HandleVisibility', 'off');

        text(ax, xText, yText, labels(k), ...
            'FontSize', 8.5, ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'BackgroundColor', 'w', ...
            'EdgeColor', [0.75 0.75 0.75], ...
            'Margin', 1.2, ...
            'Clipping', 'on');

        placedPositions(end+1,:) = bestPosition; %#ok<AGROW>
    end
end

function addQuadrantLabels(ax)
%ADDQUADRANTLABELS Identifica los cuatro modos de operación.

    xl = xlim(ax);
    yl = ylim(ax);

    xr = diff(xl);
    yr = diff(yl);

    commonArguments = { ...
        'FontSize', 9, ...
        'Color', [0.20 0.20 0.20], ...
        'BackgroundColor', 'w', ...
        'Margin', 2};

    text(ax, xl(2)-0.02*xr, yl(2)-0.04*yr, ...
        {'Cuadrante I', 'Motorización directa'}, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', commonArguments{:});

    text(ax, xl(2)-0.02*xr, yl(1)+0.04*yr, ...
        {'Cuadrante II', 'Frenado regenerativo directo'}, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'bottom', commonArguments{:});

    text(ax, xl(1)+0.02*xr, yl(1)+0.04*yr, ...
        {'Cuadrante III', 'Motorización inversa'}, ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'bottom', commonArguments{:});

    text(ax, xl(1)+0.02*xr, yl(2)-0.04*yr, ...
        {'Cuadrante IV', 'Frenado regenerativo inverso'}, ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'top', commonArguments{:});
end

function [quadrants, modes] = classifyOperatingPoint(w, T)
%CLASSIFYOPERATINGPOINT Clasifica cada punto por signos de velocidad y torque.

    w = w(:);
    T = T(:);

    quadrants = strings(size(w));
    modes = strings(size(w));

    wTolerance = max(1e-9, 1e-6 * max(abs(w)));
    tTolerance = max(1e-9, 1e-6 * max(abs(T)));

    for k = 1:numel(w)
        if abs(w(k)) <= wTolerance || abs(T(k)) <= tTolerance
            quadrants(k) = "Eje";
            modes(k) = "Transición / potencia mecánica aproximadamente nula";
        elseif w(k) > 0 && T(k) > 0
            quadrants(k) = "I";
            modes(k) = "Motorización directa";
        elseif w(k) > 0 && T(k) < 0
            quadrants(k) = "II";
            modes(k) = "Frenado regenerativo directo";
        elseif w(k) < 0 && T(k) < 0
            quadrants(k) = "III";
            modes(k) = "Motorización inversa";
        else
            quadrants(k) = "IV";
            modes(k) = "Frenado regenerativo inverso";
        end
    end
end