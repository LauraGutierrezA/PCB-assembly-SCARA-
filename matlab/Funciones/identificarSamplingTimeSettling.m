function [ts_min, ts_max, t_settling] = identificarSamplingTimeSettling(G)
    % IDENTIFICARSAMPLINGTIMESETTLING Calcula el rango de tiempo de muestreo (Ts)
    % recomendado usando el criterio del Tiempo de Asentamiento (Settling Time).
    %
    % Entrada:
    %   G          : Función de transferencia en tiempo continuo (objeto 'tf')
    %
    % Salidas:
    %   ts_min     : Límite inferior del tiempo de muestreo (s) -> 0.05 * ts
    %   ts_max     : Límite superior del tiempo de muestreo (s) -> 0.15 * ts
    %   t_settling : Tiempo de asentamiento estimado (s)

    arguments
        G % Objeto de modelo dinámico (tf, zpk, ss)
    end

    % 1. Extraer los polos para determinar el orden de la planta
    p = pole(G);
    orden = length(p);

    fprintf('\n=== Cálculo de Ts (Criterio de Tiempo de Asentamiento) ===\n');

    % 2 & 3. Calcular el tiempo de asentamiento (t_settling) según el orden
    if orden == 1
        % --- SISTEMA DE PRIMER ORDEN ---
        tau = -1 / real(p(1));
        t_settling = 4 * tau;
        
        fprintf('Orden detectado: 1er Orden | tau = %.4f s\n', tau);
        fprintf('Tiempo de asentamiento (ts = 4*tau): %.4f s\n', t_settling);

    elseif orden == 2
        % --- SISTEMA DE SEGUNDO ORDEN ---
        % La función damp entrega vectores con wn y zita para cada polo
        [wn_vec, zita_vec] = damp(G);
        
        % Tomamos los parámetros del polo dominante
        wn = wn_vec(1);
        zita = zita_vec(1);
        
        t_settling = 4 / (zita * wn);
        
        fprintf('Orden detectado: 2do Orden | zita = %.4f, wn = %.4f rad/s\n', zita, wn);
        fprintf('Tiempo de asentamiento (ts = 4/(zita*wn)): %.4f s\n', t_settling);

    else
        error('Este criterio está implementado únicamente para sistemas de 1er o 2do orden. Orden detectado: %d', orden);
    end

    % Validación de seguridad física
    if t_settling <= 0
        error('El tiempo de asentamiento calculado no es válido (ts <= 0). Verifica que el sistema sea estable.');
    end

    % 4. Calcular el rango recomendado para Ts
    ts_min = 0.05 * t_settling;
    ts_max = 0.15 * t_settling;

    % --- RESULTADOS EN CONSOLA ---
    fprintf('----------------------------------------------------------\n');
    fprintf('Rango de Ts recomendado: %.4f s  <  Ts  <  %.4f s\n', ts_min, ts_max);
    fprintf('Frecuencia de muestreo:  %.2f Hz  >  Fs  >  %.2f Hz\n', 1/ts_min, 1/ts_max);
    fprintf('==========================================================\n');
end