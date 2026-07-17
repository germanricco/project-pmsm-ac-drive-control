%% PRUEBA DE INTEGRACIÓN DEL MODELO COMPLETO

clearvars;
close all;
clc;

%% 1. Cargar parámetros

run("config/iniciar_proyecto.m");

modelo = "simulacion_sistema_completo";

%% 2. Preparar entradas nulas

t_prueba = [0; 0.05];

T_ld_prueba  = [0; 0];             % [N*m]
T_amb_prueba = [Tamb; Tamb];       % [°C]
q_ref_prueba = [0; 0];             % [rad], eje de la carga
w_ref_prueba = [0; 0];             % [rad/s], eje de la carga

entrada = Simulink.SimulationData.Dataset;

entrada = entrada.addElement( ...
    timeseries(T_ld_prueba, t_prueba), ...
    "T_ld");

entrada = entrada.addElement( ...
    timeseries(T_amb_prueba, t_prueba), ...
    "T_amb");

entrada = entrada.addElement( ...
    timeseries(q_ref_prueba, t_prueba), ...
    "q_t");

entrada = entrada.addElement( ...
    timeseries(w_ref_prueba, t_prueba), ...
    "w_t");

%% 3. Configurar simulación

in = Simulink.SimulationInput(modelo);

in = in.setExternalInput(entrada);

in = in.setModelParameter( ...
    "StopTime", ...
    string(t_prueba(end)));

%% 4. Verificar compilación del modelo

load_system(modelo);

disp("Actualizando el diagrama...");
set_param(modelo, "SimulationCommand", "update");

disp("El modelo compiló correctamente.");

%% 5. Simular

disp("Ejecutando prueba de integración...");

out = sim(in);

disp("Prueba terminada correctamente.");

%% 6. Revisar resultados guardados

disp("Señales disponibles en yout:");

nombres_salidas = out.yout.getElementNames;
disp(nombres_salidas);

fprintf("Tiempo final alcanzado: %.6f s\n", out.tout(end));

if abs(out.tout(end) - t_prueba(end)) > 1e-9
    error("La simulación no alcanzó el tiempo final esperado.");
end

disp("----------------------------------------------");
disp("MODELO LISTO PARA LAS PRIMERAS SIMULACIONES");
disp("----------------------------------------------");