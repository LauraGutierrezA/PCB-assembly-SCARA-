function R = calculadoraLPF(Fcorte, C)
    % CALCULADORALPF Calcula la resistencia necesaria para un filtro pasa bajas activo de 1er orden.
    % 
    % Entradas:
    %   Fcorte : Frecuencia de corte deseada en Hertz (Hz)
    %   C      : Capacitancia fija en Faradios (F) [Opcional, por defecto 0.1 uF = 1e-7 F]
    %
    % Salida:
    %   R      : Resistencia requerida en Ohms (Ω)
    %
    % Ejemplo de uso:
    %   R = calculadoraLPF(1000)         % Fc = 1 kHz, C = 0.1 uF por defecto
    %   R = calculadoraLPF(1000, 10e-9)  % Fc = 1 kHz, C = 10 nF

    arguments
        Fcorte (1,1) double {mustBePositive}
        C      (1,1) double {mustBePositive} = 0.1e-6 % 0.1 uF por defecto
    end

    % Cálculo de la resistencia
    R = 1 / (2 * pi * Fcorte * C);

    % Muestra un resumen en consola con unidades legibles
    fprintf('\n--- Parámetros del Filtro LPF Activo ---\n');
    fprintf('Frecuencia de corte (Fc): %.2f Hz\n', Fcorte);
    fprintf('Capacitancia fija (C):    %.2f nF (%.2f uF)\n', C * 1e9, C * 1e6);
    fprintf('Resistencia calculada:    %.2f Ω (%.2f kΩ)\n', R, R / 1e3);
    fprintf('---------------------------------------\n');
end