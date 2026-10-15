function [ts_min, ts_max, tau_eq] = identificarSamplingTimeTauEq(G)
    % IDENTIFICARSAMPLINGTIMETAUEQ Calcula el rango de tiempo de muestreo (Ts)
    % recomendado usando el criterio de la Constante de Tiempo Equivalente (Tau Eq).
    %
    % Entrada:
    %   G      : Función de transferencia en tiempo continuo (objeto 'tf')
    %
    % Salidas:
    %   ts_min : Límite inferior del tiempo de muestreo (s) -> 0.2 * Tau_eq
    %   ts_max : Límite superior del tiempo de muestreo (s) -> 0.8 * Tau_eq
    %   tau_eq : Constante de tiempo equivalente seleccionada (s)

    arguments
        G % Objeto de modelo dinámico (tf, zpk, ss)
    end

    % 1. Extraer los polos para determinar el orden y las constantes de tiempo
    % Nota: Al igual que en el método de ancho de banda, el retardo (theta) no 
    % altera los polos de la planta y es ignorado por pole(G).
    p = pole(G);
    orden = length(p);

    fprintf('\n=== Cálculo de Ts (Criterio de Tau Equivalente) ===\n');

    % 2 & 3. Determinar Tau Equivalente según el orden del sistema
    if orden == 1
        % --- SISTEMA DE PRIMER ORDEN ---
        tau_eq = -1 / real(p(1));
        fprintf('Orden detectado: 1er Orden | tau = %.4f s\n', tau_eq);

    elseif orden == 2
        % --- SISTEMA DE SEGUNDO ORDEN ---
        tau1 = -1 / real(p(1));
        tau2 = -1 / real(p(2));
        
        % Seleccionamos la constante mayor (la dinámica dominante)
        tau_eq = max(tau1, tau2);
        
        fprintf('Orden detectado: 2do Orden | tau1 = %.4f s, tau2 = %.4f s\n', tau1, tau2);
        fprintf('Tau Equivalente (tau_eq seleccionado): %.4f s\n', tau_eq);

    else
        error('Este criterio está implementado únicamente para sistemas de 1er o 2do orden. Orden detectado: %d', orden);
    end

    % Validación de seguridad física
    if tau_eq <= 0
        error('La constante de tiempo calculada no es válida (tau <= 0). Verifica la estabilidad de la planta.');
    end

    % 4. Calcular el rango recomendado para Ts
    ts_min = 0.2 * tau_eq;
    ts_max = 0.8 * tau_eq;

    % --- RESULTADOS EN CONSOLA ---
    fprintf('----------------------------------------------------------\n');
    fprintf('Rango de Ts recomendado: %.4f s  <  Ts  <  %.4f s\n', ts_min, ts_max);
    fprintf('Frecuencia de muestreo:  %.2f Hz  >  Fs  >  %.2f Hz\n', 1/ts_min, 1/ts_max);
    fprintf('==========================================================\n');
end