function [ts_min, ts_max, ws] = identificarSamplingTime(G)
    % IDENTIFICARSAMPLINGTIME Calcula el rango de tiempo de muestreo (Ts)
    % recomendado usando el método del ancho de banda (caída de -3 dB respecto a DC).
    %
    % Entrada:
    %   G      : Función de transferencia en tiempo continuo (objeto 'tf')
    %
    % Salidas:
    %   ts_min : Límite inferior del tiempo de muestreo (s) -> 2*pi / (12*ws)
    %   ts_max : Límite superior del tiempo de muestreo (s) -> 2*pi / (8*ws)
    %   ws     : Frecuencia de ancho de banda calculada (rad/s)
    
    arguments
        G % Objeto de modelo dinámico (tf, zpk, ss)
    end
    
    % 1. Extraer polos y ganancia estática (K)
    % Nota: El retardo (theta) se ignora automáticamente porque |exp(-j*w*theta)| = 1
    p = pole(G);
    orden = length(p);
    K = dcgain(G);
    
    if K == 0 || isinf(K)
        error('La ganancia estática K del sistema no es válida o es cero.');
    end
    
    fprintf('\n=== Cálculo de Tiempo de Muestreo (Método Bandwidth) ===\n');
    fprintf('Ganancia DC (K): %.4f (%.2f dB)\n', K, 20*log10(abs(K)));
    
    % 2 & 3. Calcular la frecuencia de ancho de banda (ws) según el orden
    if orden == 1
        % --- SISTEMA DE PRIMER ORDEN ---
        tau = -1 / real(p(1));
        fprintf('Orden detectado: 1er Orden | tau = %.4f s\n', tau);
        
        % La frecuencia de corte relativa a DC es independiente de K
        ws = 1 / tau;
        
    elseif orden == 2
        % --- SISTEMA DE SEGUNDO ORDEN ---
        tau1 = -1 / real(p(1));
        tau2 = -1 / real(p(2));
        fprintf('Orden detectado: 2do Orden | tau1 = %.4f s, tau2 = %.4f s\n', tau1, tau2);
        
        % Buscamos la caída de -3dB relativa a la ganancia en DC.
        % Al igualar |G(jw)|_dB = |G(0)|_dB - 3, el término K se cancela.
        % 3 - 20*log10(denom1) - 20*log10(denom2) = 0
        ecuacion = @(w) 3 - 20*log10(sqrt(1 + (w^2) * (tau1^2))) ...
                          - 20*log10(sqrt(1 + (w^2) * (tau2^2)));
                  
        % Punto inicial de búsqueda (estimación conservadora)
        w0 = 1 / max(tau1, tau2);
        
        % Resolución numérica para hallar ws
        options = optimset('Display', 'off');
        ws = fzero(ecuacion, w0, options);
        
    else
        error('Este método está implementado únicamente para sistemas de 1er o 2do orden (polos reales). Orden detectado: %d', orden);
    end
    
    % 4. Calcular el rango recomendado para Ts (Criterio de 8 a 12 muestras por ciclo)
    ts_min = (2 * pi) / (12 * ws);
    ts_max = (2 * pi) / (8 * ws);
    
    % --- RESULTADOS EN CONSOLA ---
    fprintf('Ancho de banda (ws):     %.4f rad/s (%.4f Hz)\n', ws, ws / (2*pi));
    fprintf('----------------------------------------------------------\n');
    fprintf('Rango de Ts recomendado: %.4f s  <  Ts  <  %.4f s\n', ts_min, ts_max);
    fprintf('Frecuencia de muestreo:  %.2f Hz  >  Fs  >  %.2f Hz\n', 1/ts_min, 1/ts_max);
    fprintf('==========================================================\n');
end