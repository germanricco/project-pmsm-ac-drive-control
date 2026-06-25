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

        function [fig, ax] = plotTimeSweep(obj, t, Y, paramVals, paramName, options)
            % plotTimeSweep Grafica múltiples respuestas temporales variando un parámetro.
            arguments
                obj
                t (:,1) double
                Y (:,:) double
                paramVals (1,:) double
                paramName (1,1) string
                options.Title string = "Respuesta al Escalón"
                options.WindowPosition (1,4) double = [200, 150, 700, 450]
            end
            
            numSteps = length(paramVals);
            colores = turbo(numSteps); % Genera la paleta de colores
            
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            hold(ax, 'on'); grid(ax, 'on');
            
            legendStrings = strings(numSteps, 1);
            
            for i = 1:numSteps
                plot(ax, t, Y(:, i), 'Color', colores(i, :), 'LineWidth', obj.LineWidth);
                legendStrings(i) = sprintf('%s = %0.3g', paramName, paramVals(i));
            end
            
            title(ax, sprintf('%s para distintos %s', options.Title, paramName), 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle);
            xlabel(ax, 'Tiempo (s)', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            ylabel(ax, 'Amplitud', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            set(ax, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTicks, 'GridColor', obj.GridColor, 'GridAlpha', obj.GridAlpha);
            
            legend(ax, legendStrings, 'Location', 'bestoutside', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLegend);
            hold(ax, 'off');
        end

        function [fig, ax] = plotPoleSweep(obj, polesList, paramVals, paramName, options)
            % plotPoleSweep Grafica la migración de polos variando un parámetro.
            arguments
                obj
                polesList cell
                paramVals (1,:) double
                paramName (1,1) string
                options.Title string = "Lugar de Raíces"
                options.WindowPosition (1,4) double = [250, 200, 700, 450]
            end
            
            numSteps = length(paramVals);
            colores = turbo(numSteps); % Misma paleta, sincronización perfecta
            
            fig = figure('Color', 'w', 'Position', options.WindowPosition);
            ax = axes(fig);
            hold(ax, 'on'); grid(ax, 'on');
            
            legendHandles = gobjects(numSteps, 1); 
            legendStrings = strings(numSteps, 1);
            
            % Ejes cartesianos de referencia
            xline(ax, 0, '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1, 'HandleVisibility', 'off');
            yline(ax, 0, '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1, 'HandleVisibility', 'off');
            
            for i = 1:numSteps
                p = polesList{i};
                legendHandles(i) = scatter(ax, real(p), imag(p), 60, colores(i, :), 'x', 'LineWidth', 1.5);
                legendStrings(i) = sprintf('%s = %0.3g', paramName, paramVals(i));
            end
            
            title(ax, sprintf('%s para distintos %s', options.Title, paramName), 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTitle);
            xlabel(ax, 'Eje Real (\sigma)', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            ylabel(ax, 'Eje Imaginario (j\omega)', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLabel);
            set(ax, 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeTicks, 'GridColor', obj.GridColor, 'GridAlpha', obj.GridAlpha);
            
            legend(ax, legendHandles, legendStrings, 'Location', 'bestoutside', 'FontName', obj.FontFamily, 'FontSize', obj.FontSizeLegend);
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