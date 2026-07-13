%% Comparación de i_ds(t) para condiciones iniciales +0.5 A y -0.5 A
% -------------------------------------------------------------------------
% Antes de ejecutar este script:
%   1) Abra el modelo de Simulink y cargue los parámetros de la planta.
%   2) En los integradores de i_ds de los modelos NL y LTI, escriba como
%      condición inicial la variable:
%
%                           i_ds_0
%
%   3) Verifique que el modelo registre las señales:
%         out.i_qd0_nl
%         out.i_qd0_lti
%      con el orden qd0 = [q, d, 0].
%   4) SignalPlotter.m debe encontrarse en la carpeta ../utils.
%
% El script ejecuta automáticamente dos simulaciones:
%      i_ds(0) = +0.5 A
%      i_ds(0) = -0.5 A
%
% y exporta dos archivos PDF comparando los modelos NL y LTI.
% -------------------------------------------------------------------------

close all;
clc;

%% Configuración

initialConditionVariable = 'i_ds_0';
initialConditions = [0.5, -0.5];       % [A]
stopTime = 0.05;                       % [s], suficiente para observar el decaimiento

dtMajor = 0.01;                       % marcas principales [s]
dtMinor = 0.005;                      % marcas menores [s]
closeAfterExport = false;

fileNames = {
    'comparacion_i_d_inicial_positivo.pdf'
    'comparacion_i_d_inicial_negativo.pdf'
};

plotTitles = {
    'Corriente en el eje d para i_{ds}^{r}(0)=+0.5 A'
    'Corriente en el eje d para i_{ds}^{r}(0)=-0.5 A'
};

%% Detectar el modelo de Simulink abierto

currentSystem = gcs;

if isempty(currentSystem)
    error(['No hay un modelo de Simulink activo. Abra el modelo y ', ...
           'vuelva a ejecutar el script.']);
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
                        'simulacion_nl_lti');

if ~isfolder(exportFolder)
    mkdir(exportFolder);
end

plotter = SignalPlotter(exportFolder);

%% Ejecutar los dos casos y generar las figuras

for k = 1:numel(initialConditions)

    id0 = initialConditions(k);

    fprintf('\nSimulando con i_ds(0) = %+.1f A...\n', id0);

    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setVariable(initialConditionVariable, id0);
    simIn = simIn.setModelParameter( ...
        'StartTime', '0', ...
        'StopTime', num2str(stopTime, '%.12g'));

    outCase = sim(simIn);

    % Extraer las corrientes qd0 de ambos modelos.
    [tNL, iqd0NL] = getSimSignal( ...
        getRequiredOutput(outCase, 'i_qd0_nl'));

    [tLTI, iqd0LTI] = getSimSignal( ...
        getRequiredOutput(outCase, 'i_qd0_lti'));

    requireComponents(iqd0NL, 3, 'out.i_qd0_nl');
    requireComponents(iqd0LTI, 3, 'out.i_qd0_lti');

    % Convención qd0 = [q, d, 0]: la corriente i_d es la segunda columna.
    idNL = iqd0NL(:, 2);
    idLTI = iqd0LTI(:, 2);

    % Alinear el modelo LTI con el vector temporal del modelo NL.
    idLTIonNL = alignSignal(tLTI, idLTI, tNL);

    % Gráfica con el mismo estilo del script general.
    [fig, ax] = plotter.plotTime( ...
        tNL, ...
        [idNL, idLTIonNL], ...
        Title   = plotTitles{k}, ...
        XLabel  = 'Tiempo (s)', ...
        YLabel  = 'i_{ds}^{r} (A)', ...
        Legends = {'Modelo no lineal desacoplado', ...
                   'Modelo LTI equivalente aumentado'});

    ax = resolveAxes(fig, ax);
    formatTransientAxis(ax, stopTime, dtMajor, dtMinor);

    % Línea horizontal de referencia en cero.
    if exist('yline', 'file') == 2
        yline(ax, 0, ':', 'HandleVisibility', 'off');
    end

    plotter.export(fig, fileNames{k});
    closeIfRequested(fig, closeAfterExport);

    fprintf('Figura exportada: %s\n', fullfile(exportFolder, fileNames{k}));
end

fprintf('\nGráficos generados correctamente.\n');
fprintf('Carpeta de salida:\n%s\n\n', exportFolder);


%% ========================================================================
% Funciones auxiliares locales
% ========================================================================

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


function requireComponents(y, expectedComponents, signalName)
%REQUIRECOMPONENTS Verifica la cantidad de componentes de una señal.

    actualComponents = size(y, 2);

    if actualComponents ~= expectedComponents
        error(['La señal %s debe contener %d componentes, pero se ', ...
               'encontraron %d. Revise el orden del vector qd0.'], ...
               signalName, expectedComponents, actualComponents);
    end
end


function yAligned = alignSignal(tOriginal, yOriginal, tReference)
%ALIGNSIGNAL Interpola una señal sobre un vector temporal de referencia.

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


function formatTransientAxis(ax, stopTime, dtMajor, dtMinor)
%FORMATTRANSIENTAXIS Formatea el eje temporal para observar el transitorio.

    xlim(ax, [0 stopTime]);
    xticks(ax, 0:dtMajor:stopTime);

    grid(ax, 'on');
    box(ax, 'on');

    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    ax.XMinorGrid = 'on';
    ax.YMinorGrid = 'on';

    if isprop(ax, 'XAxis') && isprop(ax.XAxis, 'MinorTickValues')
        ax.XAxis.MinorTickValues = 0:dtMinor:stopTime;
    end

    if isprop(ax, 'XAxis') && isprop(ax.XAxis, 'Exponent')
        ax.XAxis.Exponent = 0;
    end
end


function closeIfRequested(fig, closeAfterExport)
%CLOSEIFREQUESTED Cierra una figura luego de exportarla.

    if closeAfterExport && isgraphics(fig)
        close(fig);
    end
end
