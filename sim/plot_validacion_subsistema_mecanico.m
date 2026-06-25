%% Script de Validación: Subsistema Mecánico
% Requiere que la simulación de validacion_subsistema_mecanico.slx haya corrido

% 1. Extraer datos (accediendo a través del objeto 'out')
tiempo = out.sim_out.time;
theta = out.sim_out.signals.values;

% 2. Instanciar el plotter apuntando al directorio de imágenes
% (Si la carpeta no existe, la clase la creará automáticamente)
plotter = SignalPlotter("docs/img/");

% 3. Graficar usando la nueva clase
[fig, ax] = plotter.plotTime(tiempo, theta, ...
    'Title', 'Respuesta Libre del Péndulo (Condición Inicial: \pi/2)', ...
    'XLabel', 'Tiempo [s]', ...
    'YLabel', '\theta_m(t) [rad]');

% (Opcional) Si necesitas forzar los límites del eje Y que tenías antes:
ylim(ax, [-2 2]);

% 4. Exportar el gráfico como SVG vectorial
plotter.export(fig, 'fig_respuesta_libre_mecanica.svg');