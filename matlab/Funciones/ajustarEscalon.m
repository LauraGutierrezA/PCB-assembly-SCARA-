function [u_adj, y_adj, t_adj, t_step] = ajustarEscalon(u, y, t, opts)
% AJUSTARESCALON Sincroniza un experimento al origen de tiempo (t = 0)
% y elimina automáticamente valores NaN/Inf producidos por celdas vacías de Excel.
%
% Uso:
%   [u_adj, y_adj, t_adj, t_step] = ajustarEscalon(u, y, t)
%       -> Comportamiento original: detecta el primer salto en u y recorta
%          desde ahí hasta el final de los datos.
%
%   [u_adj, y_adj, t_adj, t_step] = ajustarEscalon(u, y, t, fromy=30, toy=50)
%   [u_adj, y_adj, t_adj, t_step] = ajustarEscalon(u, y, t, 'fromy', 30, 'toy', 50)
%       -> Busca el escalón en el que la ENTRADA u pasa del nivel 30 al
%          nivel 50 y recorta solo ese tramo (desde justo antes del salto
%          hasta justo antes de que u cambie de nuevo).
%
% Entradas:
%   u : Vector de entrada
%   y : Vector de salida (respuesta)
%   t : Vector de tiempo absoluto
%
% Opciones (nombre-valor):
%   fromy      : Nivel inicial de u antes del escalón
%   toy        : Nivel final de u después del escalón
%   tol        : Tolerancia para comparar niveles de u
%                (por defecto 5% de |toy - fromy|)
%   ocurrencia : Si el escalón fromy->toy aparece varias veces, cuál usar
%                (por defecto 1 = la primera)
%
% Salidas:
%   u_adj  : Vector de entrada recortado desde el escalón
%   y_adj  : Vector de salida recortado desde el escalón
%   t_adj  : Vector de tiempo normalizado (empieza estrictamente en 0.0 s)
%   t_step : Tiempo absoluto en el que ocurrió el escalón en el dataset original

arguments
    u (:,1) double
    y (:,1) double
    t (:,1) double
    opts.fromy (1,1) double = NaN
    opts.toy   (1,1) double = NaN
    opts.tol   (1,1) double = NaN
    opts.ocurrencia (1,1) double {mustBeInteger, mustBePositive} = 1
end

% 1. Asegurar vectores columna
u = u(:); y = y(:); t = t(:);

% 2. Filtrar y eliminar filas que contengan NaN o Inf
validos = isfinite(u) & isfinite(y) & isfinite(t);
u = u(validos);
y = y(validos);
t = t(validos);

if isempty(t)
    error('El vector de datos no contiene muestras válidas (todos eran NaN).');
end

usarRango = ~isnan(opts.fromy) || ~isnan(opts.toy);

if ~usarRango
    % ================= MODO ORIGINAL =================
    % 3. Detección del instante del escalón (primer salto en u)
    du_abs = [0; abs(diff(u))];
    umbral = 0.10 * max(du_abs); % 10% del cambio máximo
    idx_step = find(du_abs > umbral, 1, 'first');

    if isempty(idx_step)
        warning('No se detectó un cambio brusco en u. Se tomará idx = 1.');
        idx_step = 1;
    end

    idx_inicio = max(1, idx_step - 1);
    idx_fin    = length(t);
else
    % ================= MODO fromy -> toy =================
    if isnan(opts.fromy) || isnan(opts.toy)
        error('Debes especificar AMBOS argumentos: fromy y toy.');
    end
    if opts.fromy == opts.toy
        error('fromy y toy deben ser distintos.');
    end

    tol = opts.tol;
    if isnan(tol)
        tol = 0.05 * abs(opts.toy - opts.fromy);
    elseif tol < 0
        error('''tol'' debe ser no negativa.');
    end

    enFrom = abs(u - opts.fromy) <= tol;
    enTo   = abs(u - opts.toy)   <= tol;

    % Etiquetar muestras: 1 = nivel fromy, 2 = nivel toy, 0 = otra cosa
    etiqueta = zeros(size(u));
    etiqueta(enFrom) = 1;
    etiqueta(enTo)   = 2;

    % Trabajar solo con las muestras que están en alguno de los dos niveles.
    % Así se toleran muestras intermedias durante la transición
    % (p. ej. 30 -> 37 -> 50) que hacen que el salto no sea de una muestra.
    idxNiv = find(etiqueta > 0);
    etqNiv = etiqueta(idxNiv);
    pos = find(etqNiv(1:end-1) == 1 & etqNiv(2:end) == 2);

    lo = min(opts.fromy, opts.toy) - tol;
    hi = max(opts.fromy, opts.toy) + tol;
    candInicio = [];
    candStep   = [];
    for k = pos.'
        a = idxNiv(k);       % última muestra en fromy
        b = idxNiv(k+1);     % primera muestra en toy
        intermedias = u(a+1:b-1);
        % Las muestras de en medio deben ser solo transición entre ambos
        % niveles (no otro escalón distinto, p. ej. 30 -> 10 -> 50)
        if all(intermedias >= lo & intermedias <= hi)
            candInicio(end+1) = a; %#ok<AGROW>
            candStep(end+1)   = b; %#ok<AGROW>
        end
    end

    if isempty(candStep)
        error(['No se encontró un escalón de u = %.4g a u = %.4g ' ...
               '(tol = %.4g). Niveles presentes en u (redondeados): %s'], ...
               opts.fromy, opts.toy, tol, mat2str(unique(round(u, 2)).'));
    end
    if opts.ocurrencia > numel(candStep)
        error('Solo hay %d escalón(es) %.4g -> %.4g; pediste la ocurrencia %d.', ...
              numel(candStep), opts.fromy, opts.toy, opts.ocurrencia);
    end

    idx_inicio = candInicio(opts.ocurrencia);   % último punto en el nivel fromy
    idx_step   = candStep(opts.ocurrencia);     % primer punto en el nivel toy

    % Fin del tramo: última muestra antes de que u salga del nivel toy
    salida = find(~enTo(idx_step:end), 1, 'first');
    if isempty(salida)
        idx_fin = length(t);
    else
        idx_fin = idx_step + salida - 2;
    end
end

t_step = t(idx_inicio);

% 4. Recorte y normalización de tiempo a t = 0
u_adj = u(idx_inicio:idx_fin);
y_adj = y(idx_inicio:idx_fin);
t_adj = t(idx_inicio:idx_fin) - t_step;

if usarRango
    fprintf('Escalón %.4g -> %.4g ajustado: %d puntos | t = 0.00 s a %.2f s (t_abs_inicio = %.2f s)\n', ...
        opts.fromy, opts.toy, length(t_adj), t_adj(end), t_step);
else
    fprintf('Escalón ajustado exitosamente: %d puntos válidos | t = 0.00 s a %.2f s (t_abs_inicio = %.2f s)\n', ...
        length(t_adj), t_adj(end), t_step);
end
end