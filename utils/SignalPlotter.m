classdef SignalPlotter < handle
    % SIGNALPLOTTER Clase profesional para la visualización y exportación de señales.
    % Diseñada para uso a largo plazo (5-10 años) con soporte multifase y temas.
    
    properties
        FontFamily (1,1) string = "Times New Roman"
        LineWidth  (1,1) double = 1.5
        FontSizeTitle (1,1) double = 14
        FontSizeLabel (1,1) double = 12
        FontSizeTicks (1,1) double = 10
        FontSizeLegend (1,1) double = 11
        GridColor     (1,3) double = [0.85, 0.85, 0.85]
        GridAlpha     (1,1) double = 0.6
        MarginY       (1,1) double = 0.15 % 15% de margen dinámico
        ExportPath    (1,1) string = "./"
        
        % Paleta de colores corporativa/académica (HEX convertible a RGB)
        ColorPalette cell = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F'}
    end
    
    methods
        function obj = SignalPlotter(exportPath)
            % Constructor de la clase
            if nargin > 0
                obj.ExportPath = exportPath;
                if ~exist(obj.ExportPath, 'dir')
                    mkdir(obj.ExportPath);
                end
            end
        end
        
        %%
        function [fig, ax] = plotTime(obj, t, Y, options)
            % plotTime Grafica una o múltiples señales en el dominio del tiempo.
            % Y puede ser un vector columna o una matriz donde cada columna es una fase.
            arguments
                obj
                t (:,1) double % Enfuerza vector columna
                Y (:,:) double % Matriz de datos [muestras x canales]
                options.Title string = "Señales en el Dominio del Tiempo"
                options.XLabel string = "Tiempo (s)"
                options.YLabel string = "Amplitud"
                options.Legends cell = {}
                options.WindowPosition (1,4) double = [200, 150, 850, 420]
            end
            
            % Validación de dimensiones
            if size(Y, 1) ~= length(t)
                % Si el usuario pasó la matriz transpuesta, la corregimos silenciosamente
                if size(Y, 2) == length(t)
                    Y = Y';
                else
                    error("SignalPlotter:numel", "El número de filas de Y debe coincidir con la longitud de t.");
                end
            end
            
            numSignals = size(Y, 2);
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            hold(ax, 'on');
            
            % Ploteo dinámico de señales
            for idx = 1:numSignals
                colorIdx = mod(idx - 1, length(obj.ColorPalette)) + 1;
                plot(ax, t, Y(:, idx), ...
                    'Color', obj.ColorPalette{colorIdx}, ...
                    'LineWidth', obj.LineWidth, ...
                    'LineStyle', '-');
            end
            
            % Línea de referencia en cero si cruza el eje
            minGlobal = min(Y(:));
            maxGlobal = max(Y(:));
            if minGlobal < 0 && maxGlobal > 0
                yline(ax, 0, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.8, 'HandleVisibility', 'off');
            end
            
            % Formateo estético de los ejes
            grid(ax, 'on');
            xlim(ax, [min(t), max(t)]);
            
            rangoY = maxGlobal - minGlobal;
            if rangoY == 0, rangoY = 1; end
            ylim(ax, [minGlobal - (obj.MarginY * rangoY), maxGlobal + (obj.MarginY * rangoY)]);
            
            % Aplicar estilos del objeto
            set(ax, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTicks, ...
                    'GridColor', obj.GridColor, 'GridAlpha', obj.GridAlpha);
                
            title(ax, options.Title, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle, 'FontWeight', 'bold');
            xlabel(ax, options.XLabel, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            ylabel(ax, options.YLabel, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            
            if ~isempty(options.Legends)
                legend(ax, options.Legends, 'Location', 'best', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLegend);
            end
            
            hold(ax, 'off');
        end

        %%
        function [fig, ax] = plotTimeStairs(obj, t, Y, options)
            % plotTimeStairs Grafica una o múltiples señales escalonadas en el tiempo.
            arguments
                obj
                t (:,1) double
                Y (:,:) double
                options.Title string = "Señales Escalonadas en el Dominio del Tiempo"
                options.XLabel string = "Tiempo (s)"
                options.YLabel string = "Amplitud"
                options.Legends cell = {}
                options.WindowPosition (1,4) double = [200, 150, 850, 420]
            end
        
            % Validación de dimensiones
            if size(Y, 1) ~= length(t)
                if size(Y, 2) == length(t)
                    Y = Y';
                else
                    error("SignalPlotter:numel", ...
                        "El número de filas de Y debe coincidir con la longitud de t.");
                end
            end
        
            numSignals = size(Y, 2);
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            hold(ax, 'on');
        
            % Ploteo escalonado
            for idx = 1:numSignals
                colorIdx = mod(idx - 1, length(obj.ColorPalette)) + 1;
                stairs(ax, t, Y(:, idx), ...
                    'Color', obj.ColorPalette{colorIdx}, ...
                    'LineWidth', obj.LineWidth, ...
                    'LineStyle', '-');
            end
        
            % Línea de referencia en cero si cruza el eje
            minGlobal = min(Y(:));
            maxGlobal = max(Y(:));
            if minGlobal < 0 && maxGlobal > 0
                yline(ax, 0, '--', 'Color', [0.3 0.3 0.3], ...
                    'LineWidth', 0.8, 'HandleVisibility', 'off');
            end
        
            % Formato de ejes
            grid(ax, 'on');
            xlim(ax, [min(t), max(t)]);
        
            rangoY = maxGlobal - minGlobal;
            if rangoY == 0
                rangoY = 1;
            end
            ylim(ax, [minGlobal - (obj.MarginY * rangoY), ...
                      maxGlobal + (obj.MarginY * rangoY)]);
        
            set(ax, 'FontName', obj.FontFamily, ...
                    'FontSize', obj.FontSizeTicks, ...
                    'GridColor', obj.GridColor, ...
                    'GridAlpha', obj.GridAlpha);
        
            title(ax, options.Title, ...
                'FontName', obj.FontFamily, ...
                'FontSize', obj.FontSizeTitle, ...
                'FontWeight', 'bold');
        
            xlabel(ax, options.XLabel, ...
                'FontName', obj.FontFamily, ...
                'FontSize', obj.FontSizeLabel);
        
            ylabel(ax, options.YLabel, ...
                'FontName', obj.FontFamily, ...
                'FontSize', obj.FontSizeLabel);
        
            if ~isempty(options.Legends)
                legend(ax, options.Legends, ...
                    'Location', 'best', ...
                    'FontName', obj.FontFamily, ...
                    'FontSize', obj.FontSizeLegend);
            end
        
            hold(ax, 'off');
        end

        %%
        function [fig, ax] = plotSpectrum(obj, y, fs, options)
            % plotSpectrum Calcula y grafica el espectro de magnitud unilateral nativamente
            arguments
                obj
                y (:,1) double
                fs (1,1) double
                options.Title string = "Espectro de Magnitud (FFT Unilateral)"
                options.FMax double = fs/2
                options.WindowPosition (1,4) double = [200, 150, 850, 420]
            end
            
            % Algoritmo nativo FFT
            L = length(y);
            NFFT = 2^nextpow2(L);
            Y_fft = fft(y, NFFT) / L;
            f = fs * (0:(NFFT/2)) / NFFT;
            amplitude = 2 * abs(Y_fft(1:NFFT/2+1));
            amplitude(1) = amplitude(1) / 2; % Componente de continua corregida
            
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            
            plot(ax, f, amplitude, 'Color', obj.ColorPalette{1}, 'LineWidth', obj.LineWidth);
            grid(ax, 'on');
            xlim(ax, [0, options.FMax]);
            
            maxAmp = max(amplitude);
            if maxAmp == 0, maxAmp = 1; end
            ylim(ax, [0, maxAmp * (1 + obj.MarginY)]);
            
            set(ax, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTicks, ...
                    'GridColor', obj.GridColor, 'GridAlpha', obj.GridAlpha);
                
            title(ax, options.Title, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle, 'FontWeight', 'bold');
            xlabel(ax, "Frecuencia (Hz)", 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            ylabel(ax, "|Y(f)|", 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
        end

        %%
        function [fig, ax] = plotTimeSweep(obj, t, Y, paramVals, paramName, options)
            % plotTimeSweep Grafica múltiples respuestas temporales variando un parámetro.
            arguments
                obj
                t (:,1) double
                Y (:,:) double
                paramVals (1,:) double
                paramName (1,1) string
                options.Title string = "" % Dejado en blanco por defecto para mejor control
                options.WindowPosition (1,4) double = [200, 150, 850, 500]
            end
            
            numSteps = length(paramVals);
            
            % --- LÓGICA DE COLORES INTELIGENTE ---
            % Si son pocas curvas, usamos la paleta categórica de alto contraste.
            % Si son muchas, usamos un gradiente continuo elegante (parula).
            if numSteps <= length(obj.ColorPalette)
                colores = obj.ColorPalette(1:numSteps); % Toma colores distintos (HEX)
            else
                matrizRGB = parula(numSteps);
                colores = num2cell(matrizRGB, 2);       % Convierte a celdas RGB
            end
            
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            hold(ax, 'on'); grid(ax, 'on');
            
            legendStrings = strings(numSteps, 1);
            
            for i = 1:numSteps
                % Graficar usando la celda de color correspondiente
                plot(ax, t, Y(:, i), 'Color', colores{i}, 'LineWidth', obj.LineWidth);
                legendStrings(i) = sprintf('%s = %0.3g', paramName, paramVals(i));
            end
            
            % --- LÓGICA DE TÍTULO ---
            if options.Title == ""
                tituloFinal = sprintf('Análisis de Sensibilidad variando %s', paramName);
            else
                tituloFinal = options.Title;
            end
            
            title(ax, tituloFinal, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle, 'FontWeight', 'bold');
            xlabel(ax, 'Tiempo (s)', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            ylabel(ax, 'Amplitud', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            set(ax, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTicks, 'GridColor', obj.GridColor, 'GridAlpha', obj.GridAlpha);
            
            legend(ax, legendStrings, 'Location', 'bestoutside', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLegend);
            hold(ax, 'off');
        end

        %%
        function [fig, ax] = plotPoleSweep(obj, polesList, paramVals, paramName, options)
            % plotPoleSweep Grafica la migración de polos de forma robusta.
            arguments
                obj
                polesList cell
                paramVals (1,:) double
                paramName (1,1) string
                options.Title string = ""
                options.WindowPosition (1,4) double = [200, 200, 750, 480]
                options.ShowColorbar (1,1) logical = true
            end
            
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            hold(ax, 'on'); grid(ax, 'on');
            
            % Configuración de estilo global (Texto normal)
            set(ax, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTicks, ...
                    'GridColor', obj.GridColor, 'GridAlpha', obj.GridAlpha);
            
            % Ejes cartesianos de referencia
            xline(ax, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');
            yline(ax, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');
            
            numSteps = length(paramVals);
            
            % LÓGICA 1: CASO NOMINAL (1 SOLO PUNTO)
            if numSteps == 1
                p = polesList{1};
                scatter(ax, real(p), imag(p), 120, 'x', 'MarkerEdgeColor', obj.ColorPalette{1}, ...
                        'LineWidth', 2, 'DisplayName', 'Polos Dominantes');
                
                if options.Title ~= ""
                    title(ax, options.Title, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle, 'FontWeight', 'bold');
                end
                
            % LÓGICA 2: MIGRACIÓN DE POLOS (BARRIDO MULTIPLE)
            else
                colormap(ax, turbo(numSteps));
                colores = turbo(numSteps);
                numPoles = length(polesList{1});
                
                % 1. Dibujar línea punteada de trayectoria
                for pIdx = 1:numPoles
                    trayectoria = zeros(numSteps, 1);
                    for step = 1:numSteps
                       trayectoria(step) = polesList{step}(pIdx);
                    end
                    plot(ax, real(trayectoria), imag(trayectoria), '--', ...
                         'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'HandleVisibility', 'off');
                end
                
                % 2. Dibujar las cruces variando el color
                for i = 1:numSteps
                    p = polesList{i};
                    scatter(ax, real(p), imag(p), 100, 'x', 'MarkerEdgeColor', colores(i, :), ...
                            'LineWidth', 1.8, 'HandleVisibility', 'off');
                end
                
                % 3. Insertar la Barra de Color (Colorbar)
                if options.ShowColorbar
                    cb = colorbar(ax);
                    clim(ax, [min(paramVals), max(paramVals)]);
                    cb.Label.String = paramName; 
                    cb.Label.FontName = obj.FontFamily;
                    cb.Label.FontSize = obj.FontSizeLabel;
                end
                
                if options.Title ~= ""
                    title(ax, options.Title, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle, 'FontWeight', 'bold');
                end
            end
            
            % Etiquetas de ejes (MATLAB soporta \sigma y \omega por defecto sin modo LaTeX)
            xlabel(ax, 'Eje Real (\sigma)', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            ylabel(ax, 'Eje Imaginario (j\omega)', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            
            hold(ax, 'off');
        end
        
        function export(obj, figHandle, filename)
            % export Guarda la figura en formato vectorial o imagen
            fullPath = fullfile(obj.ExportPath, filename);
            [~, ~, ext] = fileparts(filename);
            
            if lower(ext) == ".pdf"
                exportgraphics(figHandle, fullPath, 'ContentType', 'vector', 'BackgroundColor', 'none');
            elseif lower(ext) == ".svg"
                % Comando específico para SVG vectorial puro
                print(figHandle, fullPath, '-dsvg', '-vector');
            elseif lower(ext) == ".png"
                exportgraphics(figHandle, fullPath, 'Resolution', 300);
            else
                exportgraphics(figHandle, fullPath);
            end
            fprintf("SignalPlotter: Figura exportada exitosamente en: %s\n", fullPath);
        end
    end
end