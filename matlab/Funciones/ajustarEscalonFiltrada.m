function [u_adj, y_adj, t_adj, t_step, y_raw] = ajustarEscalonFiltrada(u, y, t, opts)
% AJUSTARESCALONFILTRADA Igual que ajustarEscalon, pero además filtra la
% salida y para quitar ruido y picos antes de identificar.
%
% El filtro se aplica sobre la señal COMPLETA (antes de recortar), para
% evitar efectos de borde justo en el inicio del escalón. Todos los
% filtros son de fase cero (centrados), así que NO introducen retardo
% artificial (no inflan theta).
%
% Uso:
%   [u2,y2,t2,ts] = ajustarEscalonFiltrada(u, y, t)
%   [u2,y2,t2,ts] = ajustarEscalonFiltrada(u, y, t, fromy=30, toy=50)
%   [u2,y2,t2,ts,y_raw] = ajustarEscalonFiltrada(u, y, t, fromy=30, toy=50, ...
%                              metodo="movmedian", ventana=9, graficar=true)
%
% Opciones del escalón (se pasan tal cual a ajustarEscalon):
%   fromy, toy, tol, ocurrencia
%
% Opciones del filtro:
%   metodo   : "movmedian" (defecto) -> elimina picos/outliers sin redondear el escalón
%              "medmean"            -> movmedian y luego movmean (más suave)
%              "movmean"            -> promedio móvil centrado
%              "butter"             -> Butterworth pasa-bajas con filtfilt
%                                      (requiere Signal Processing Toolbox)
%              "none"               -> sin filtro
%   ventana  : Tamaño de ventana en MUESTRAS para movmedian/movmean (defecto 7)
%   fc       : Frecuencia de corte en Hz para "butter" (defecto 5 Hz)
%   orden    : Orden del Butterworth (defecto 2)
%   graficar : true para ver señal original vs filtrada (defecto false)
%
% Salidas:
%   u_adj, t_adj, t_step : igual que ajustarEscalon
%   y_adj : salida FILTRADA y recortada
%   y_raw : salida ORIGINAL (sin filtrar) recortada, mismo tamaño que y_adj

arguments
    u (:,1) double
    y (:,1) double
    t (:,1) double
    % --- Escalón ---
    opts.fromy (1,1) double = NaN
    opts.toy   (1,1) double = NaN
    opts.tol   (1,1) double = NaN
    opts.ocurrencia (1,1) double = 1
    % --- Filtro ---
    opts.metodo (1,1) string {mustBeMember(opts.metodo, ...
        ["movmedian","medmean","movmean","butter","none"])} = "movmedian"
    opts.ventana (1,1) double = 7
    opts.fc      (1,1) double = 5
    opts.orden   (1,1) double = 2
    opts.graficar (1,1) logical = false
end

% 1. Limpiar NaN/Inf aquí también, para que el filtro trabaje sobre datos válidos
validos = isfinite(u) & isfinite(y) & isfinite(t);
u = u(validos); y = y(validos); t = t(validos);
if isempty(t)
    error('El vector de datos no contiene muestras válidas.');
end

% 2. Filtrar la señal completa
switch opts.metodo
    case "movmedian"
        y_f = movmedian(y, opts.ventana);
    case "medmean"
        y_f = movmean(movmedian(y, opts.ventana), opts.ventana);
    case "movmean"
        y_f = movmean(y, opts.ventana);
    case "butter"
        if ~exist('butter', 'file') || ~exist('filtfilt', 'file')
            error('El método "butter" requiere la Signal Processing Toolbox.');
        end
        Ts = median(diff(t));
        fs = 1 / Ts;
        if opts.fc >= fs/2
            error('fc = %.3g Hz debe ser menor que fs/2 = %.3g Hz.', opts.fc, fs/2);
        end
        [b, a] = butter(opts.orden, opts.fc / (fs/2));
        y_f = filtfilt(b, a, y);
    case "none"
        y_f = y;
end

% 3. Recortar con ajustarEscalon (señal filtrada)
nv = {'fromy', opts.fromy, 'toy', opts.toy, 'tol', opts.tol, ...
      'ocurrencia', opts.ocurrencia};
[u_adj, y_adj, t_adj, t_step] = ajustarEscalon(u, y_f, t, nv{:});

% 4. Recortar también la señal original (mismo tramo) para comparar
%    (evalc solo silencia el mensaje repetido de ajustarEscalon)
y_raw = []; %#ok<NASGU>
evalc('[~, y_raw] = ajustarEscalon(u, y, t, nv{:});');

fprintf('Filtro aplicado: %s', opts.metodo);
if any(opts.metodo == ["movmedian","medmean","movmean"])
    fprintf(' (ventana = %d muestras)', opts.ventana);
elseif opts.metodo == "butter"
    fprintf(' (fc = %.3g Hz, orden %d)', opts.fc, opts.orden);
end
fprintf('\n');

% 5. Gráfica opcional
if opts.graficar
    figure('Name', 'ajustarEscalonFiltrada');
    plot(t_adj, y_raw, 'Color', [0.6 0.6 0.6]); hold on;
    plot(t_adj, y_adj, 'b', 'LineWidth', 1.5);
    yyaxis right; plot(t_adj, u_adj, 'Color', [0.85 0.33 0.1]); ylabel('u');
    yyaxis left; ylabel('y');
    grid on; xlabel('Tiempo (s)');
    legend('y original', 'y filtrada', 'u', 'Location', 'best');
    title(sprintf('Escalón ajustado y filtrado (%s)', opts.metodo));
end
end