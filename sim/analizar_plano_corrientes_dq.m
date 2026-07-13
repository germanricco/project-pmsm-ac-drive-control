%% Analisis y graficacion de la trayectoria de corrientes en el plano d-q
% -------------------------------------------------------------------------
% Requisitos:
%   1) Ejecutar primero el modelo de Simulink para obtener la variable out.
%   2) out debe contener:
%        i_qd0_nl  -> [i_q, i_d, i_0] del modelo no lineal
%        i_qd0_lti -> [i_q, i_d, i_0] del modelo LTI
%   3) Opcionalmente:
%        v_qs_sim y T_ld_sim para marcar los cambios de entrada.
%
% Resultados:
%   - plano_corrientes_dq_fisico.pdf
%   - plano_corrientes_dq_zoom_numerico.pdf
%   - resumen_plano_corrientes_dq.csv
%   - muestras_cambios_plano_corrientes_dq.csv
%
% No se utiliza clear ni clear all para conservar los parametros de planta.
% -------------------------------------------------------------------------

close all;
clc;

%% Configuracion

qd0Order = [1 2 3];          % Convencion: [q d 0]
exportPDF = true;
closeAfterExport = false;

% Escala minima del eje d para la figura fisica.
% Con i_q del orden de 10 A, 0.5 A permite mostrar que i_d es despreciable
% sin exagerar residuos numericos del solver.
minPhysicalIdHalfRange = 0.5; % [A]

% Porcentaje del maximo |i_q| usado para definir una escala fisica del eje d.
physicalIdFractionOfIq = 0.05;

% Factor para pasar de A a microamperes en la ampliacion numerica.
A_to_uA = 1e6;

%% Verificar la salida de Simulink

if ~exist('out', 'var')
    error(['No existe la variable "out" en el workspace. ', ...
           'Ejecuta primero el modelo de Simulink.']);
end

%% Carpeta de exportacion

scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder)
    scriptFolder = pwd;
end

exportFolder = fullfile(scriptFolder, '..', 'docs', 'img', ...
                        'simulacion_nl_lti');

if ~isfolder(exportFolder)
    mkdir(exportFolder);
end

%% Extraer corrientes qd0

[tNL, iqd0NL] = getSimSignalDQ(getRequiredOutputDQ(out, 'i_qd0_nl'));
[tLTI, iqd0LTI] = getSimSignalDQ(getRequiredOutputDQ(out, 'i_qd0_lti'));

requireComponentsDQ(iqd0NL, 3, 'out.i_qd0_nl');
requireComponentsDQ(iqd0LTI, 3, 'out.i_qd0_lti');

iqd0NL = iqd0NL(:, qd0Order);
iqd0LTI = iqd0LTI(:, qd0Order);

% Convencion adoptada:
% columna 1 -> q
% columna 2 -> d
% columna 3 -> 0
iqNL = iqd0NL(:, 1);
idNL = iqd0NL(:, 2);

% Alinear el modelo LTI al vector temporal del modelo NL.
iqd0LTIonNL = alignSignalDQ(tLTI, iqd0LTI, tNL);
iqLTI = iqd0LTIonNL(:, 1);
idLTI = iqd0LTIonNL(:, 2);

%% Detectar cambios de las entradas, si fueron registradas

changeTimesVqs = [];
changeTimesTld = [];

[hasVqs, vqsSignal] = tryGetOutputDQ(out, 'v_qs_sim');
if hasVqs
    [tVqs, vqs] = getSimSignalDQ(vqsSignal);
    changeTimesVqs = detectChangeTimesDQ(tVqs, vqs);
end

[hasTld, TldSignal] = tryGetOutputDQ(out, 'T_ld_sim');
if hasTld
    [tTld, Tld] = getSimSignalDQ(TldSignal);
    changeTimesTld = detectChangeTimesDQ(tTld, Tld);
end

changeTimes = unique([changeTimesVqs(:); changeTimesTld(:)]);
changeTimes = changeTimes(changeTimes >= tNL(1) & changeTimes <= tNL(end));

% Se incluyen el inicio y el final para identificar el sentido temporal.
markTimes = unique([tNL(1); changeTimes(:); tNL(end)]);
idxMark = nearestIndexesDQ(tNL, markTimes);

%% Calcular indicadores cuantitativos

iMagNL = hypot(idNL, iqNL);
iMagLTI = hypot(idLTI, iqLTI);

maxAbsIdNL = max(abs(idNL));
maxAbsIdLTI = max(abs(idLTI));

rmsIdNL = sqrt(mean(idNL.^2));
rmsIdLTI = sqrt(mean(idLTI.^2));

maxAbsIqNL = max(abs(iqNL));
maxAbsIqLTI = max(abs(iqLTI));

maxMagNL = max(iMagNL);
maxMagLTI = max(iMagLTI);

ratioIdIqNL = safeRatioDQ(maxAbsIdNL, maxAbsIqNL);
ratioIdIqLTI = safeRatioDQ(maxAbsIdLTI, maxAbsIqLTI);

rmseId = sqrt(mean((idNL - idLTI).^2));
rmseIq = sqrt(mean((iqNL - iqLTI).^2));

maxAbsIqAll = max([maxAbsIqNL, maxAbsIqLTI]);
maxAbsIdAll = max([maxAbsIdNL, maxAbsIdLTI]);

%% Figura 1: escala fisicamente significativa

figPhysical = figure( ...
    'Color', 'w', ...
    'Name', 'Plano d-q - escala fisica', ...
    'Position', [100 100 900 680]);

ax = axes(figPhysical);
hold(ax, 'on');

hNL = plot(ax, idNL, iqNL, ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Modelo no lineal');

hLTI = plot(ax, idLTI, iqLTI, '--', ...
    'LineWidth', 1.6, ...
    'DisplayName', 'Modelo LTI');

% Ejes de referencia
xline(ax, 0, ':', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(ax, 0, ':', 'LineWidth', 1.0, 'HandleVisibility', 'off');

% Puntos correspondientes a cambios de entrada.
if ~isempty(idxMark)
    hChanges = plot(ax, idNL(idxMark), iqNL(idxMark), 'o', ...
        'MarkerSize', 6, ...
        'LineWidth', 1.0, ...
        'MarkerFaceColor', 'w', ...
        'DisplayName', 'Inicio, cambios y final');
else
    hChanges = gobjects(0);
end

% Inicio y fin con marcadores diferentes.
hStart = plot(ax, idNL(1), iqNL(1), 's', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.2, ...
    'MarkerFaceColor', 'w', ...
    'DisplayName', 'Inicio');

hEnd = plot(ax, idNL(end), iqNL(end), 'd', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.2, ...
    'MarkerFaceColor', 'w', ...
    'DisplayName', 'Final');

% Limites del eje d:
% 1) nunca menores al rango fisico minimo;
% 2) proporcionales a la corriente q;
% 3) suficientemente amplios si i_d deja de ser despreciable.
idHalfRange = max([ ...
    minPhysicalIdHalfRange, ...
    physicalIdFractionOfIq * maxAbsIqAll, ...
    1.20 * maxAbsIdAll]);

if idHalfRange <= 0
    idHalfRange = minPhysicalIdHalfRange;
end
xlim(ax, [-idHalfRange, idHalfRange]);

% Limites del eje q con margen.
setPaddedYLimitsDQ(ax, [iqNL; iqLTI], 0.08);

title(ax, 'Trayectoria parametrica del vector de corriente en el plano d-q');
xlabel(ax, 'i_{ds}^{r} (A)');
ylabel(ax, 'i_{qs}^{r} (A)');

grid(ax, 'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
ax.FontSize = 11;
box(ax, 'on');

legendHandles = [hNL, hLTI, hChanges, hStart, hEnd];
legendHandles = legendHandles(isgraphics(legendHandles));
legend(ax, legendHandles, 'Location', 'best');

% Nota breve dentro de la figura.
annotationText = sprintf([ ...
    'max |i_d| NL = %.3e A\n', ...
    'max |i_d| LTI = %.3e A'], ...
    maxAbsIdNL, maxAbsIdLTI);

text(ax, 0.02, 0.03, annotationText, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', ...
    'Margin', 5, ...
    'FontSize', 9);

if exportPDF
    physicalFile = fullfile(exportFolder, ...
        'plano_corrientes_dq_fisico.pdf');
    exportgraphics(figPhysical, physicalFile, ...
        'ContentType', 'vector');
end

%% Figura 2: ampliacion del residuo numerico de i_d

% Esta figura NO representa una excursion fisicamente relevante.
% El eje horizontal se expresa en microamperes para evitar la notacion
% automatica x10^-7 y dejar claro el orden de magnitud.

figZoom = figure( ...
    'Color', 'w', ...
    'Name', 'Plano d-q - ampliacion numerica', ...
    'Position', [140 120 900 680]);

axZoom = axes(figZoom);
hold(axZoom, 'on');

idNL_uA = idNL * A_to_uA;
idLTI_uA = idLTI * A_to_uA;

plot(axZoom, idNL_uA, iqNL, ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Modelo no lineal');

plot(axZoom, idLTI_uA, iqLTI, '--', ...
    'LineWidth', 1.6, ...
    'DisplayName', 'Modelo LTI');

xline(axZoom, 0, ':', 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
yline(axZoom, 0, ':', 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');

if ~isempty(idxMark)
    plot(axZoom, idNL_uA(idxMark), iqNL(idxMark), 'o', ...
        'MarkerSize', 6, ...
        'LineWidth', 1.0, ...
        'MarkerFaceColor', 'w', ...
        'DisplayName', 'Inicio, cambios y final');
end

plot(axZoom, idNL_uA(1), iqNL(1), 's', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.2, ...
    'MarkerFaceColor', 'w', ...
    'DisplayName', 'Inicio');

plot(axZoom, idNL_uA(end), iqNL(end), 'd', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.2, ...
    'MarkerFaceColor', 'w', ...
    'DisplayName', 'Final');

setPaddedXLimitsDQ(axZoom, [idNL_uA; idLTI_uA], 0.15, 0.05);
setPaddedYLimitsDQ(axZoom, [iqNL; iqLTI], 0.08);

title(axZoom, ...
    'Ampliacion numerica de la corriente en el eje d');
xlabel(axZoom, 'i_{ds}^{r} (\muA)');
ylabel(axZoom, 'i_{qs}^{r} (A)');

grid(axZoom, 'on');
axZoom.XMinorGrid = 'on';
axZoom.YMinorGrid = 'on';
axZoom.XMinorTick = 'on';
axZoom.YMinorTick = 'on';
axZoom.FontSize = 11;
box(axZoom, 'on');
legend(axZoom, 'Location', 'best');

zoomNote = sprintf([ ...
    'El eje d esta ampliado: las variaciones son del orden de microamperes.\n', ...
    'No deben interpretarse como excursiones fisicamente significativas.']);

text(axZoom, 0.02, 0.03, zoomNote, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', ...
    'Margin', 5, ...
    'FontSize', 9);

if exportPDF
    zoomFile = fullfile(exportFolder, ...
        'plano_corrientes_dq_zoom_numerico.pdf');
    exportgraphics(figZoom, zoomFile, ...
        'ContentType', 'vector');
end

%% Tabla de indicadores

summaryTable = table( ...
    ["Modelo no lineal"; "Modelo LTI"], ...
    [maxAbsIdNL; maxAbsIdLTI], ...
    [rmsIdNL; rmsIdLTI], ...
    [maxAbsIqNL; maxAbsIqLTI], ...
    [maxMagNL; maxMagLTI], ...
    [ratioIdIqNL; ratioIdIqLTI], ...
    'VariableNames', { ...
        'Modelo', ...
        'MaxAbsId_A', ...
        'RmsId_A', ...
        'MaxAbsIq_A', ...
        'MaxMagnitudCorriente_A', ...
        'RelacionMaxIdMaxIq'});

comparisonTable = table( ...
    rmseId, rmseIq, ...
    'VariableNames', {'RMSE_Id_A', 'RMSE_Iq_A'});

fprintf('\n============================================================\n');
fprintf('ANALISIS DE LA TRAYECTORIA DE CORRIENTES EN EL PLANO d-q\n');
fprintf('============================================================\n\n');

disp(summaryTable);

fprintf('Comparacion NL - LTI:\n');
disp(comparisonTable);

fprintf('Interpretacion preliminar:\n');
fprintf(['  Si RelacionMaxIdMaxIq << 1, la corriente i_d es ', ...
         'despreciable frente a i_q.\n']);
fprintf(['  La figura fisica debe verse practicamente vertical ', ...
         'sobre i_d = 0.\n']);
fprintf(['  La figura ampliada solo muestra residuos numericos o ', ...
         'pequenos errores de desacoplamiento.\n\n']);

%% Tabla en los instantes de cambio

markTable = table( ...
    markTimes, ...
    idNL(idxMark), ...
    iqNL(idxMark), ...
    idLTI(idxMark), ...
    iqLTI(idxMark), ...
    'VariableNames', { ...
        'Tiempo_s', ...
        'Id_NL_A', ...
        'Iq_NL_A', ...
        'Id_LTI_A', ...
        'Iq_LTI_A'});

fprintf('Valores en el inicio, cambios de entrada y final:\n');
disp(markTable);

%% Exportar tablas para el analisis posterior

summaryFile = fullfile(exportFolder, ...
    'resumen_plano_corrientes_dq.csv');
writetable(summaryTable, summaryFile);

marksFile = fullfile(exportFolder, ...
    'muestras_cambios_plano_corrientes_dq.csv');
writetable(markTable, marksFile);

fprintf('\nArchivos generados en:\n%s\n\n', exportFolder);

if closeAfterExport
    close(figPhysical);
    close(figZoom);
end

%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

function signal = getRequiredOutputDQ(out, signalName)
%GETREQUIREDOUTPUTDQ Obtiene una salida obligatoria.

    [found, signal] = tryGetOutputDQ(out, signalName);

    if ~found
        error('No se encontro la senal obligatoria out.%s.', signalName);
    end
end


function [found, signal] = tryGetOutputDQ(out, signalName)
%TRYGETOUTPUTDQ Obtiene una senal de struct o SimulationOutput.

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


function [t, y] = getSimSignalDQ(signal)
%GETSIMSIGNALDQ Extrae tiempo y datos de formatos habituales de Simulink.

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
        error(['Formato de senal no reconocido. Configura los bloques ', ...
               'To Workspace como Timeseries o Structure With Time.']);
    end

    t = double(t(:));
    y = double(y);
    y = orientSamplesByRowsDQ(y, numel(t));

    if any(~isfinite(t))
        error('El vector temporal contiene NaN o Inf.');
    end

    if any(~isfinite(y), 'all')
        error('La senal contiene NaN o Inf.');
    end
end


function y = orientSamplesByRowsDQ(y, numberOfSamples)
%ORIENTSAMPLESBYROWSDQ Coloca la dimension temporal en las filas.

    if isempty(y)
        error('La senal no contiene datos.');
    end

    if numel(y) == numberOfSamples
        y = reshape(y, numberOfSamples, 1);
        return;
    end

    dimensions = size(y);
    timeDimension = find(dimensions == numberOfSamples, 1, 'first');

    if isempty(timeDimension)
        error(['No se pudo identificar la dimension temporal. ', ...
               'Se esperaban %d muestras y el tamano recibido fue [%s].'], ...
               numberOfSamples, num2str(dimensions));
    end

    permutation = [timeDimension, ...
                   setdiff(1:ndims(y), timeDimension, 'stable')];

    y = permute(y, permutation);
    y = reshape(y, numberOfSamples, []);
end


function requireComponentsDQ(y, expectedComponents, signalName)
%REQUIRECOMPONENTSDQ Verifica la cantidad de componentes.

    actualComponents = size(y, 2);

    if actualComponents ~= expectedComponents
        error(['La senal %s debe contener %d componentes, ', ...
               'pero se encontraron %d. Revisa el orden del Mux.'], ...
               signalName, expectedComponents, actualComponents);
    end
end


function yAligned = alignSignalDQ(tOriginal, yOriginal, tReference)
%ALIGNSIGNALDQ Interpola una senal sobre un tiempo de referencia.

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


function changeTimes = detectChangeTimesDQ(t, y)
%DETECTCHANGETIMESDQ Detecta cambios de una entrada por tramos.

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


function indexes = nearestIndexesDQ(t, targetTimes)
%NEARESTINDEXESDQ Devuelve el indice mas cercano a cada tiempo pedido.

    t = t(:);
    targetTimes = targetTimes(:);
    indexes = zeros(size(targetTimes));

    for k = 1:numel(targetTimes)
        [~, indexes(k)] = min(abs(t - targetTimes(k)));
    end

    indexes = unique(indexes, 'stable');
end


function ratio = safeRatioDQ(numerator, denominator)
%SAFERATIODQ Evita division por cero.

    if denominator > eps
        ratio = numerator / denominator;
    else
        ratio = NaN;
    end
end


function setPaddedYLimitsDQ(ax, values, relativePadding)
%SETPADDEDYLIMITSDQ Ajusta limites verticales con margen.

    values = values(isfinite(values));

    if isempty(values)
        ylim(ax, [-1 1]);
        return;
    end

    valueMin = min(values);
    valueMax = max(values);
    valueRange = valueMax - valueMin;

    if valueRange <= eps(max(1, max(abs(values))))
        padding = max(1, abs(valueMax)) * relativePadding;
    else
        padding = relativePadding * valueRange;
    end

    ylim(ax, [valueMin - padding, valueMax + padding]);
end


function setPaddedXLimitsDQ(ax, values, relativePadding, minimumHalfRange)
%SETPADDEDXLIMITSDQ Ajusta limites horizontales con margen.

    values = values(isfinite(values));

    if isempty(values)
        xlim(ax, [-minimumHalfRange, minimumHalfRange]);
        return;
    end

    valueMin = min(values);
    valueMax = max(values);
    valueRange = valueMax - valueMin;

    if valueRange <= eps(max(1, max(abs(values))))
        center = 0.5 * (valueMin + valueMax);
        halfRange = minimumHalfRange;
        xlim(ax, [center - halfRange, center + halfRange]);
    else
        padding = relativePadding * valueRange;
        xlim(ax, [valueMin - padding, valueMax + padding]);
    end
end
