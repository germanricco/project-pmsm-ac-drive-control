%% Análisis automático de transitorios del accionamiento PMSM
% -------------------------------------------------------------------------
% Objetivo:
%   - Analizar únicamente las señales principales de la simulación.
%   - Detectar automáticamente los cambios de v_qs y T_ld.
%   - Calcular valores iniciales/finales, tiempo de crecimiento 10-90 %,
%     tiempo de establecimiento ±1 % y sobrepaso.
%   - Generar tablas para incorporar al informe.
%
% Requisito:
%   Ejecutar primero el modelo de Simulink para obtener la variable "out".
%
% Señales esperadas dentro de out:
%   w_m_nl      : velocidad angular del rotor
%   i_qd0_nl    : corrientes [q d 0]
%   T_m_nl      : torque electromagnético
%   v_qs_sim    : tensión de consigna de eje q
%   T_ld_sim    : torque externo aplicado en el eje de carga
%
% Convención:
%   El valor final de cada tramo se estima como la mediana de la ventana
%   final del intervalo comprendido entre dos cambios sucesivos de entrada.
% -------------------------------------------------------------------------

close all;
clc;

%% Configuración

qd0Order = [1 2 3];         % [q d 0]
settlingBand = 0.01;        % banda de establecimiento ±1 %
preWindow = 0.010;          % ventana previa para estimar el valor inicial [s]
finalFraction = 0.15;       % fracción final del intervalo usada para y_f
minFinalWindow = 0.010;     % ventana final mínima [s]
maxFinalWindow = 0.050;     % ventana final máxima [s]
eventTolerance = 1e-8;      % tolerancia para fusionar eventos simultáneos [s]
exportCSV = true;

if ~exist('out', 'var')
    error(['No existe la variable "out" en el workspace. ', ...
           'Ejecuta primero el modelo de Simulink.']);
end

%% Extraer señales

[tW, w] = getSimSignal(getRequiredOutput(out, 'w_m_nl'));
[tIqd0, iqd0] = getSimSignal(getRequiredOutput(out, 'i_qd0_nl'));
[tTm, Tm] = getSimSignal(getRequiredOutput(out, 'T_m_nl'));
[tVqs, vqs] = getSimSignal(getRequiredOutput(out, 'v_qs_sim'));
[tTld, Tld] = getSimSignal(getRequiredOutput(out, 'T_ld_sim'));

requireComponents(iqd0, 3, 'out.i_qd0_nl');
iqd0 = iqd0(:, qd0Order);
iq = iqd0(:, 1);

% Asegurar señales escalares.
w = w(:,1);
Tm = Tm(:,1);
vqs = vqs(:,1);
Tld = Tld(:,1);

%% Detectar cambios de las entradas

changeTimesVqs = detectChangeTimes(tVqs, vqs);
changeTimesTld = detectChangeTimes(tTld, Tld);
changeTimes = mergeCloseTimes([changeTimesVqs(:); changeTimesTld(:)], ...
                              eventTolerance);

if isempty(changeTimes)
    error('No se detectaron cambios en v_qs_sim ni en T_ld_sim.');
end

simulationEnd = min([tW(end), tIqd0(end), tTm(end), tVqs(end), tTld(end)]);
changeTimes = changeTimes(changeTimes < simulationEnd);

numberOfEvents = numel(changeTimes);

%% Reservar resultados

Evento = (1:numberOfEvents).';
t_k_s = changeTimes(:);
Entrada = strings(numberOfEvents,1);

vqs_i_V = nan(numberOfEvents,1);
vqs_f_V = nan(numberOfEvents,1);
Tld_i_Nm = nan(numberOfEvents,1);
Tld_f_Nm = nan(numberOfEvents,1);

w_i_rad_s = nan(numberOfEvents,1);
w_f_rad_s = nan(numberOfEvents,1);
iq_i_A = nan(numberOfEvents,1);
iq_f_A = nan(numberOfEvents,1);
Tm_i_Nm = nan(numberOfEvents,1);
Tm_f_Nm = nan(numberOfEvents,1);

tr_w_ms = nan(numberOfEvents,1);
ts_w_ms = nan(numberOfEvents,1);
Mp_w_rad_s = nan(numberOfEvents,1);
Mp_w_pct = nan(numberOfEvents,1);

t10_w_s = nan(numberOfEvents,1);
t90_w_s = nan(numberOfEvents,1);

tR_iq_ms = nan(numberOfEvents,1);
tS_iq_ms = nan(numberOfEvents,1);
Mp_iq_A = nan(numberOfEvents,1);
Mp_iq_pct = nan(numberOfEvents,1);

t10_iq_s = nan(numberOfEvents,1);
t90_iq_s = nan(numberOfEvents,1);

tr_Tm_ms = nan(numberOfEvents,1);
ts_Tm_ms = nan(numberOfEvents,1);
Mp_Tm_Nm = nan(numberOfEvents,1);
Mp_Tm_pct = nan(numberOfEvents,1);

changedVqs = false(numberOfEvents,1);
changedTld = false(numberOfEvents,1);

%% Analizar cada intervalo

for k = 1:numberOfEvents
    tk = changeTimes(k);

    if k < numberOfEvents
        tEnd = changeTimes(k+1);
    else
        tEnd = simulationEnd;
    end

    changedVqs(k) = any(abs(changeTimesVqs - tk) <= eventTolerance);
    changedTld(k) = any(abs(changeTimesTld - tk) <= eventTolerance);

    if changedVqs(k) && changedTld(k)
        Entrada(k) = "v_qs y T_ld";
    elseif changedVqs(k)
        Entrada(k) = "v_qs";
    elseif changedTld(k)
        Entrada(k) = "T_ld";
    else
        Entrada(k) = "evento";
    end

    [vqs_i_V(k), vqs_f_V(k)] = inputBeforeAfter(tVqs, vqs, tk);
    [Tld_i_Nm(k), Tld_f_Nm(k)] = inputBeforeAfter(tTld, Tld, tk);

    mw = computeTransientMetrics(tW, w, tk, tEnd, preWindow, ...
        finalFraction, minFinalWindow, maxFinalWindow, settlingBand);

    miq = computeTransientMetrics(tIqd0, iq, tk, tEnd, preWindow, ...
        finalFraction, minFinalWindow, maxFinalWindow, settlingBand);

    mTm = computeTransientMetrics(tTm, Tm, tk, tEnd, preWindow, ...
        finalFraction, minFinalWindow, maxFinalWindow, settlingBand);

    w_i_rad_s(k) = mw.yInitial;
    w_f_rad_s(k) = mw.yFinal;
    tr_w_ms(k) = 1e3 * mw.riseTime;
    ts_w_ms(k) = 1e3 * mw.settlingTime;
    Mp_w_rad_s(k) = mw.overshootAbs;
    Mp_w_pct(k) = mw.overshootPct;
    t10_w_s(k) = mw.t10;
    t90_w_s(k) = mw.t90;

    iq_i_A(k) = miq.yInitial;
    iq_f_A(k) = miq.yFinal;
    tR_iq_ms(k) = 1e3 * miq.riseTime;
    tS_iq_ms(k) = 1e3 * miq.settlingTime;
    Mp_iq_A(k) = miq.overshootAbs;
    Mp_iq_pct(k) = miq.overshootPct;
    t10_iq_s(k) = miq.t10;
    t90_iq_s(k) = miq.t90;

    Tm_i_Nm(k) = mTm.yInitial;
    Tm_f_Nm(k) = mTm.yFinal;
    tr_Tm_ms(k) = 1e3 * mTm.riseTime;
    ts_Tm_ms(k) = 1e3 * mTm.settlingTime;
    Mp_Tm_Nm(k) = mTm.overshootAbs;
    Mp_Tm_pct(k) = mTm.overshootPct;
end

%% Tabla general: valores iniciales y finales de todos los transitorios

resultadosTodos = table( ...
    Evento, t_k_s, Entrada, ...
    vqs_i_V, vqs_f_V, Tld_i_Nm, Tld_f_Nm, ...
    w_i_rad_s, w_f_rad_s, iq_i_A, iq_f_A, Tm_i_Nm, Tm_f_Nm);

%% Tabla principal 1: velocidad frente a escalones de tensión

metricasVelocidad = table( ...
    Evento(changedVqs), ...
    t_k_s(changedVqs), ...
    vqs_i_V(changedVqs), ...
    vqs_f_V(changedVqs), ...
    w_i_rad_s(changedVqs), ...
    w_f_rad_s(changedVqs), ...
    tr_w_ms(changedVqs), ...
    ts_w_ms(changedVqs), ...
    Mp_w_rad_s(changedVqs), ...
    Mp_w_pct(changedVqs), ...
    'VariableNames', { ...
        'Evento','t_k_s','vqs_i_V','vqs_f_V', ...
        'w_i_rad_s','w_f_rad_s','t_crecimiento_ms', ...
        't_establecimiento_ms','sobrepaso_rad_s','sobrepaso_pct'});

%% Tabla principal 2: corriente y torque frente a escalones de carga

metricasCorrienteTorque = table( ...
    Evento(changedTld), ...
    t_k_s(changedTld), ...
    Tld_i_Nm(changedTld), ...
    Tld_f_Nm(changedTld), ...
    iq_i_A(changedTld), ...
    iq_f_A(changedTld), ...
    tR_iq_ms(changedTld), ...
    tS_iq_ms(changedTld), ...
    Mp_iq_A(changedTld), ...
    Mp_iq_pct(changedTld), ...
    Tm_i_Nm(changedTld), ...
    Tm_f_Nm(changedTld), ...
    tr_Tm_ms(changedTld), ...
    ts_Tm_ms(changedTld), ...
    Mp_Tm_Nm(changedTld), ...
    Mp_Tm_pct(changedTld), ...
    'VariableNames', { ...
        'Evento','t_k_s','Tld_i_Nm','Tld_f_Nm', ...
        'iq_i_A','iq_f_A','t_crecimiento_iq_ms', ...
        't_establecimiento_iq_ms','sobrepaso_iq_A', ...
        'sobrepaso_iq_pct','Tm_i_Nm','Tm_f_Nm', ...
        't_crecimiento_Tm_ms','t_establecimiento_Tm_ms', ...
        'sobrepaso_Tm_Nm','sobrepaso_Tm_pct'});

%% Mostrar resultados

fprintf('\n============================================================\n');
fprintf('VALORES INICIALES Y FINALES DE TODOS LOS TRANSITORIOS\n');
fprintf('============================================================\n');
disp(resultadosTodos);

fprintf('\n============================================================\n');
fprintf('VELOCIDAD FRENTE A CAMBIOS DE v_qs\n');
fprintf('============================================================\n');
disp(metricasVelocidad);

fprintf('\n============================================================\n');
fprintf('CORRIENTE Y TORQUE FRENTE A CAMBIOS DE T_ld\n');
fprintf('============================================================\n');
disp(metricasCorrienteTorque);

%% Exportar resultados

if exportCSV
    scriptFolder = fileparts(mfilename('fullpath'));
    if isempty(scriptFolder)
        scriptFolder = pwd;
    end

    outputFolder = fullfile(scriptFolder, 'resultados_transitorios');
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end

    writetable(resultadosTodos, ...
        fullfile(outputFolder, 'resultados_transitorios_todos.csv'));
    writetable(metricasVelocidad, ...
        fullfile(outputFolder, 'metricas_velocidad_vqs.csv'));
    writetable(metricasCorrienteTorque, ...
        fullfile(outputFolder, 'metricas_corriente_torque_Tld.csv'));

    save(fullfile(outputFolder, 'resultados_transitorios.mat'), ...
        'resultadosTodos', 'metricasVelocidad', ...
        'metricasCorrienteTorque');

    fprintf('\nResultados exportados en:\n%s\n', outputFolder);
end

%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

function signal = getRequiredOutput(out, signalName)
    [found, signal] = tryGetOutput(out, signalName);
    if ~found
        error('No se encontró la señal obligatoria out.%s.', signalName);
    end
end

function [found, signal] = tryGetOutput(out, signalName)
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
    if isobject(signal) && isprop(signal, 'Values')
        signal = signal.Values;
    end

    if isa(signal, 'timeseries')
        t = signal.Time;
        y = signal.Data;
    elseif isstruct(signal) && isfield(signal, 'time') && isfield(signal, 'signals')
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
end

function y = orientSamplesByRows(y, numberOfSamples)
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
    actualComponents = size(y, 2);
    if actualComponents ~= expectedComponents
        error(['La señal %s debe contener %d componentes, ', ...
               'pero se encontraron %d.'], ...
               signalName, expectedComponents, actualComponents);
    end
end

function changeTimes = detectChangeTimes(t, y)
    t = t(:);
    y = y(:,1);
    dy = diff(y);

    scale = max(1, max(abs(y)));
    tolerance = 1e-10 * scale;
    indexes = find(abs(dy) > tolerance);

    changeTimes = t(indexes + 1);
    changeTimes = changeTimes(changeTimes > min(t));
end

function merged = mergeCloseTimes(times, tolerance)
    times = sort(times(:));
    if isempty(times)
        merged = times;
        return;
    end

    merged = times(1);
    for k = 2:numel(times)
        if abs(times(k) - merged(end)) <= tolerance
            merged(end) = mean([merged(end), times(k)]);
        else
            merged(end+1,1) = times(k); %#ok<AGROW>
        end
    end
end

function [valueBefore, valueAfter] = inputBeforeAfter(t, y, tk)
    t = t(:);
    y = y(:,1);

    idxBefore = find(t < tk, 1, 'last');
    idxAfter = find(t >= tk, 1, 'first');

    if isempty(idxBefore)
        valueBefore = y(1);
    else
        valueBefore = y(idxBefore);
    end

    if isempty(idxAfter)
        valueAfter = y(end);
    else
        valueAfter = y(idxAfter);
    end
end

function metrics = computeTransientMetrics(t, y, tk, tEnd, preWindow, ...
        finalFraction, minFinalWindow, maxFinalWindow, settlingBand)

    t = t(:);
    y = y(:,1);

    idxSegment = (t >= tk) & (t < tEnd);
    tSegment = t(idxSegment);
    ySegment = y(idxSegment);

    if numel(tSegment) < 3
        metrics = emptyMetrics();
        return;
    end

    idxPre = (t >= max(t(1), tk - preWindow)) & (t < tk);
    if any(idxPre)
        yInitial = median(y(idxPre), 'omitnan');
    else
        idxBefore = find(t < tk, 1, 'last');
        if isempty(idxBefore)
            yInitial = ySegment(1);
        else
            yInitial = y(idxBefore);
        end
    end

    segmentDuration = tEnd - tk;
    finalWindow = min(max(finalFraction * segmentDuration, minFinalWindow), ...
                      maxFinalWindow);

    idxFinal = tSegment >= (tEnd - finalWindow);
    if ~any(idxFinal)
        idxFinal = max(1, floor(0.85*numel(tSegment))):numel(tSegment);
    end
    yFinal = median(ySegment(idxFinal), 'omitnan');

    delta = yFinal - yInitial;
    amplitudeTolerance = 1e-10 * max([1, abs(yInitial), abs(yFinal)]);

    metrics = emptyMetrics();
    metrics.yInitial = yInitial;
    metrics.yFinal = yFinal;

    if abs(delta) <= amplitudeTolerance
        metrics.peakValue = yFinal;
        metrics.overshootAbs = 0;
        metrics.overshootPct = 0;
        return;
    end

    y10 = yInitial + 0.10 * delta;
    y90 = yInitial + 0.90 * delta;

    if delta > 0
        t10 = firstCrossingTime(tSegment, ySegment, y10, +1);
        t90 = firstCrossingTime(tSegment, ySegment, y90, +1);
        peakValue = max(ySegment);
        overshootAbs = max(0, peakValue - yFinal);
    else
        t10 = firstCrossingTime(tSegment, ySegment, y10, -1);
        t90 = firstCrossingTime(tSegment, ySegment, y90, -1);
        peakValue = min(ySegment);
        overshootAbs = max(0, yFinal - peakValue);
    end

    if isfinite(t10) && isfinite(t90) && t90 >= t10
        riseTime = t90 - t10;
    else
        riseTime = NaN;
    end

    band = settlingBand * abs(delta);
    insideBand = abs(ySegment - yFinal) <= band;
    lastOutside = find(~insideBand, 1, 'last');

    if isempty(lastOutside)
        settlingTime = 0;
    elseif lastOutside < numel(tSegment)
        settlingTime = tSegment(lastOutside + 1) - tk;
    else
        settlingTime = NaN;
    end

    metrics.y10 = y10;
    metrics.y90 = y90;
    metrics.t10 = t10;
    metrics.t90 = t90;
    metrics.riseTime = riseTime;
    metrics.settlingTime = settlingTime;
    metrics.peakValue = peakValue;
    metrics.overshootAbs = overshootAbs;
    metrics.overshootPct = 100 * overshootAbs / abs(delta);
end

function crossingTime = firstCrossingTime(t, y, target, direction)
    if direction > 0
        index = find(y >= target, 1, 'first');
    else
        index = find(y <= target, 1, 'first');
    end

    if isempty(index)
        crossingTime = NaN;
        return;
    end

    if index == 1
        crossingTime = t(1);
        return;
    end

    t1 = t(index-1);
    t2 = t(index);
    y1 = y(index-1);
    y2 = y(index);

    if y2 == y1
        crossingTime = t2;
    else
        crossingTime = t1 + (target - y1) * (t2 - t1) / (y2 - y1);
    end
end

function metrics = emptyMetrics()
    metrics = struct( ...
        'yInitial', NaN, ...
        'yFinal', NaN, ...
        'y10', NaN, ...
        'y90', NaN, ...
        't10', NaN, ...
        't90', NaN, ...
        'riseTime', NaN, ...
        'settlingTime', NaN, ...
        'peakValue', NaN, ...
        'overshootAbs', NaN, ...
        'overshootPct', NaN);
end
