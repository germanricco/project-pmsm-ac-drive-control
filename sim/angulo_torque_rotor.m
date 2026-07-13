%% Evolución del ángulo de torque del rotor
% -------------------------------------------------------------------------
% Ejecutar primero el modelo de Simulink para obtener la variable "out".
%
% Señales obligatorias:
%   out.v_abc_nl    -> tensiones trifásicas [va vb vc]
%   out.theta_m_nl  -> posición mecánica del rotor [rad]
%   out.T_m_nl      -> torque electromagnético [N·m]
%
% Señal opcional:
%   out.v_qd0_nl    -> tensiones [vq vd v0], utilizada para verificar
%                     el cálculo obtenido desde abc.
%
% El script:
%   1) calcula theta_ev desde las tensiones abc;
%   2) calcula theta_r = Pp*theta_m;
%   3) obtiene delta = theta_ev - theta_r en [-pi, pi];
%   4) compara el signo de delta con el torque electromagnético;
%   5) exporta las figuras en PDF.
%
% No se usa clear ni clear all para conservar los parámetros de la planta.
% -------------------------------------------------------------------------

close all;
clc;

%% Configuración

exportPDF       = true;
closeAfterExport = false;
showVerification = true;

% Pares de polos. Si existe Pp en el workspace y es válido, se utiliza.
% En caso contrario se adopta el valor de la guía del proyecto.
PpAngle = 3;

if exist('Pp', 'var') && isnumeric(Pp) && isscalar(Pp) && ...
        isfinite(Pp) && Pp > 0
    PpAngle = Pp;
else
    warning('Pp no existe o no es válido. Se utiliza PpAngle = 3.');
end

% Orden de las componentes dentro de los vectores exportados.
abcOrder = [1 2 3];  % [a b c]
qd0Order = [1 2 3];  % [q d 0]

% Umbral relativo para considerar nulo el vector de tensión.
voltageThresholdFactor = 1e-8;

%% Verificar la salida de Simulink

if ~exist('out', 'var')
    error(['No existe la variable "out" en el workspace. ', ...
           'Ejecuta primero el modelo de Simulink.']);
end

%% Extraer señales obligatorias

[tVabc, vabc] = getSimSignal(getRequiredOutput(out, 'v_abc_nl'));
[tTheta, thetaM] = getSimSignal(getRequiredOutput(out, 'theta_m_nl'));
[tTm, Tm] = getSimSignal(getRequiredOutput(out, 'T_m_nl'));

requireComponents(vabc, 3, 'out.v_abc_nl');

vabc = vabc(:, abcOrder);

% Alinear posición y torque con el vector temporal de las tensiones abc.
thetaM = alignSignal(tTheta, thetaM, tVabc);
Tm     = alignSignal(tTm, Tm, tVabc);

thetaM = thetaM(:, 1);
Tm     = Tm(:, 1);

t = tVabc;
va = vabc(:, 1);
vb = vabc(:, 2);
vc = vabc(:, 3);

%% Transformación de Clarke: abc -> alpha-beta

vAlpha = (2/3) .* (va - 0.5.*vb - 0.5.*vc);

vBeta = (2/3) .* ( ...
    (sqrt(3)/2).*vb - (sqrt(3)/2).*vc);

voltageMagnitude = hypot(vAlpha, vBeta);

%% Ángulos eléctricos

% Ángulo instantáneo del vector de tensión del estator.
thetaEv = atan2(vBeta, vAlpha);

% Ángulo eléctrico del rotor.
thetaR = PpAngle .* thetaM;

% Diferencia angular llevada al intervalo [-pi, pi].
deltaABC = wrapToPiLocal(thetaEv - thetaR);

% El ángulo no está definido cuando el vector de tensión es prácticamente nulo.
referenceMagnitude = max(voltageMagnitude);

if referenceMagnitude <= eps
    error(['La magnitud de out.v_abc_nl es prácticamente nula durante ', ...
           'toda la simulación. No puede calcularse el ángulo de torque.']);
end

voltageThreshold = voltageThresholdFactor * referenceMagnitude;
validVoltage = voltageMagnitude > voltageThreshold;

deltaABC(~validVoltage) = NaN;
deltaABCDeg = rad2deg(deltaABC);

%% Verificación opcional mediante las tensiones qd0

hasVqd0 = false;
deltaQD0 = [];
deltaQD0Deg = [];
angleErrorDeg = [];

[hasVqd0, vqd0Signal] = tryGetOutput(out, 'v_qd0_nl');

if hasVqd0
    [tVqd0, vqd0] = getSimSignal(vqd0Signal);
    requireComponents(vqd0, 3, 'out.v_qd0_nl');

    vqd0 = vqd0(:, qd0Order);
    vqd0 = alignSignal(tVqd0, vqd0, t);

    vq = vqd0(:, 1);
    vd = vqd0(:, 2);

    % Para la convención de Park empleada en el proyecto:
    % delta = atan2(-vd, vq)
    deltaQD0 = atan2(-vd, vq);

    qdMagnitude = hypot(vq, vd);
    validQD0 = qdMagnitude > voltageThreshold;

    deltaQD0(~validQD0) = NaN;
    deltaQD0Deg = rad2deg(deltaQD0);

    validComparison = isfinite(deltaABC) & isfinite(deltaQD0);

    if any(validComparison)
        angleError = wrapToPiLocal( ...
            deltaABC(validComparison) - deltaQD0(validComparison));

        angleErrorDeg = rad2deg(angleError);

        fprintf('\nVerificación abc frente a qd0:\n');
        fprintf('  Error RMS:    %.6g grados\n', ...
                sqrt(mean(angleErrorDeg.^2)));
        fprintf('  Error máximo: %.6g grados\n', ...
                max(abs(angleErrorDeg)));
    end
elseif showVerification
    warning(['No se encontró out.v_qd0_nl. Se omite la verificación ', ...
             'del ángulo mediante coordenadas qd0.']);
end

%% Verificación de signo entre delta y torque

torqueTolerance = 1e-6 * max(1, max(abs(Tm)));
angleTolerance  = deg2rad(0.1);

validSign = isfinite(deltaABC) & ...
            abs(Tm) > torqueTolerance & ...
            abs(deltaABC) > angleTolerance;

if any(validSign)
    signAgreement = mean( ...
        sign(deltaABC(validSign)) == sign(Tm(validSign))) * 100;

    fprintf('\nCoincidencia de signo entre delta y T_m: %.2f %%\n', ...
            signAgreement);
else
    signAgreement = NaN;
    warning(['No hay suficientes muestras válidas para comparar ', ...
             'el signo de delta con el torque.']);
end

%% Carpeta de exportación

scriptFolder = fileparts(mfilename('fullpath'));

if isempty(scriptFolder)
    scriptFolder = pwd;
end

exportFolder = fullfile(scriptFolder, '..', 'docs', 'img', ...
                        'simulacion_nl_lti');

if exportPDF && ~isfolder(exportFolder)
    mkdir(exportFolder);
end

%% Figura 1: evolución del ángulo de torque

fig1 = figure('Color', 'w');
ax1 = axes(fig1);

plot(ax1, t, deltaABCDeg, 'LineWidth', 1.5);

xlabel(ax1, 'Tiempo (s)');
ylabel(ax1, '\delta (°)');
title(ax1, 'Evolución del ángulo de torque del rotor');

grid(ax1, 'on');
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
ax1.XMinorTick = 'on';
ax1.YMinorTick = 'on';
box(ax1, 'on');

xlim(ax1, [min(t), max(t)]);
ylim(ax1, [-180, 180]);
yticks(ax1, -180:45:180);
yline(ax1, 0, ':', 'HandleVisibility', 'off');

if exportPDF
    exportgraphics(fig1, ...
        fullfile(exportFolder, 'angulo_torque_rotor.pdf'), ...
        'ContentType', 'vector');
end

%% Figura 2: relación entre delta y torque electromagnético

fig2 = figure('Color', 'w');
layout = tiledlayout(fig2, 2, 1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

ax21 = nexttile(layout);
plot(ax21, t, deltaABCDeg, 'LineWidth', 1.5);
ylabel(ax21, '\delta (°)');
title(ax21, 'Ángulo de torque y torque electromagnético');
grid(ax21, 'on');
ax21.XMinorGrid = 'on';
ax21.YMinorGrid = 'on';
ax21.XMinorTick = 'on';
ax21.YMinorTick = 'on';
box(ax21, 'on');
ylim(ax21, [-180, 180]);
yticks(ax21, -180:45:180);
yline(ax21, 0, ':', 'HandleVisibility', 'off');

ax22 = nexttile(layout);
plot(ax22, t, Tm, 'LineWidth', 1.5);
xlabel(ax22, 'Tiempo (s)');
ylabel(ax22, 'T_m (N·m)');
grid(ax22, 'on');
ax22.XMinorGrid = 'on';
ax22.YMinorGrid = 'on';
ax22.XMinorTick = 'on';
ax22.YMinorTick = 'on';
box(ax22, 'on');
yline(ax22, 0, ':', 'HandleVisibility', 'off');

linkaxes([ax21, ax22], 'x');
xlim(ax21, [min(t), max(t)]);

if exportPDF
    exportgraphics(fig2, ...
        fullfile(exportFolder, 'angulo_torque_y_torque.pdf'), ...
        'ContentType', 'vector');
end

%% Figura 3: verificación abc frente a qd0

if showVerification && hasVqd0 && ~isempty(deltaQD0Deg)
    fig3 = figure('Color', 'w');
    ax3 = axes(fig3);

    hold(ax3, 'on');
    plot(ax3, t, deltaABCDeg, ...
        'LineWidth', 1.5, ...
        'DisplayName', 'Desde tensiones abc');

    plot(ax3, t, deltaQD0Deg, '--', ...
        'LineWidth', 1.3, ...
        'DisplayName', 'Desde tensiones qd0');

    xlabel(ax3, 'Tiempo (s)');
    ylabel(ax3, '\delta (°)');
    title(ax3, 'Verificación del cálculo del ángulo de torque');

    grid(ax3, 'on');
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    ax3.XMinorTick = 'on';
    ax3.YMinorTick = 'on';
    box(ax3, 'on');

    xlim(ax3, [min(t), max(t)]);
    ylim(ax3, [-180, 180]);
    yticks(ax3, -180:45:180);
    yline(ax3, 0, ':', 'HandleVisibility', 'off');
    legend(ax3, 'Location', 'best');

    if exportPDF
        exportgraphics(fig3, ...
            fullfile(exportFolder, 'verificacion_angulo_torque.pdf'), ...
            'ContentType', 'vector');
    end
end

%% Finalización

fprintf('\nCálculo del ángulo de torque finalizado.\n');

if exportPDF
    fprintf('Figuras exportadas en:\n%s\n\n', exportFolder);
end

if closeAfterExport
    close all;
end


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
        error(['Formato de señal no reconocido. Configura el bloque ', ...
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
               'Se esperaban %d muestras y se recibió [%s].'], ...
               numberOfSamples, num2str(dimensions));
    end

    permutation = [timeDimension, ...
                   setdiff(1:ndims(y), timeDimension, 'stable')];

    y = permute(y, permutation);
    y = reshape(y, numberOfSamples, []);
end


function requireComponents(y, expectedComponents, signalName)
%REQUIRECOMPONENTS Verifica la cantidad de componentes de una señal.

    actualComponents = size(y, 2);

    if actualComponents ~= expectedComponents
        error(['La señal %s debe contener %d componentes, ', ...
               'pero se encontraron %d.'], ...
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


function angleWrapped = wrapToPiLocal(angle)
%WRAPTOPILOCAL Lleva un ángulo al intervalo [-pi, pi].

    angleWrapped = atan2(sin(angle), cos(angle));
end
