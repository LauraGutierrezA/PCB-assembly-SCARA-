function [rangos_K_final, es_estable] = controllerRouth(G_disc, K_test)
    % CONTROLLERROUTH Discrete stability via Bilinear Transform and Routh Array.
    % Muestra las desigualdades paso a paso y rastrea el rango de Kp.
    %
    % Entradas:
    %   G_disc : Función de transferencia discreta (z)
    %   K_test : Valor de Kp para probar estabilidad final (Default: 1)
    
    arguments
        G_disc
        K_test (1,1) double = 1
    end
    fprintf('\n=== ROUTH STABILITY ANALYSIS (BILINEAR TRANSFORM) ===\n');
    
    % 1. Preparar polinomios en z
    if hasdelay(G_disc), G_disc = delay2z(G_disc); end
    [num, den] = tfdata(G_disc, 'v');
    
    n_den = length(den); n_num = length(num);
    if n_den > n_num, num = [zeros(1, n_den - n_num), num];
    elseif n_num > n_den, den = [zeros(1, n_num - n_den), den]; end
    n = length(den) - 1;
    
    fprintf('1. Characteristic equation Q(z) = Den(z) + K*Num(z) = 0\n');
    fprintf('   Detected system degree in z (n) = %d\n\n', n);
    
    % 2. Transformación Bilineal: z = (w+1)/(w-1)
    % Multiplicamos por (w-1)^n para limpiar el denominador y dejar Q(w) polinomial
    syms w K real
    fprintf('2. Applying Bilinear Transformation: z = (w+1)/(w-1)\n');
    
    Q_w = sym(0);
    for i = 0:n
        % El coeficiente de z^(n-i)
        coef_z = den(i+1) + K * num(i+1);
        power_z = n - i;
        % Transformación escalada: (w+1)^power_z * (w-1)^(n - power_z)
        Q_w = Q_w + coef_z * (w + 1)^power_z * (w - 1)^(n - power_z);
    end
    Q_w = expand(Q_w);
    
    % Extraer los coeficientes de w (desde w^n hasta w^0)
    C_w = coeffs(Q_w, w, 'All');
    pad_len = (n + 1) - length(C_w);
    if pad_len > 0
        C_w = [sym(zeros(1, pad_len)), C_w];
    end
    d = simplify(C_w);
    
    fprintf('   Q(w) generated successfully. Degree in w = %d\n\n', n);
    fprintf('3. Constructing Routh Array and evaluating inequalities...\n\n');
    
    % 3. Construcción del Arreglo de Routh
    cols = ceil((n+1)/2);
    R = sym(zeros(n+1, cols));
    
    % Fila 1 y 2
    idx1 = 1:2:(n+1);
    idx2 = 2:2:(n+1);
    R(1, 1:length(idx1)) = d(idx1);
    R(2, 1:length(idx2)) = d(idx2);
    
    % Llenar el resto de la matriz
    for i = 3:(n+1)
        for j = 1:(cols-1)
            num_term = R(i-1, 1)*R(i-2, j+1) - R(i-2, 1)*R(i-1, j+1);
            den_term = R(i-1, 1);
            % Usamos simplifyFraction para evitar la explosión simbólica
            R(i, j) = simplifyFraction(num_term / den_term);
        end
    end
    
    % 4. Extraer primera columna y evaluar desigualdades (C1 > 0)
    C1 = R(:, 1);
    rangos_K = [-Inf, Inf]; % Rango inicial válido (Todo el dominio)
    
    for i = 1:(n+1)
        expr = C1(i);
        
        % Forzar el primer elemento a ser positivo si no depende de K
        if isempty(symvar(expr))
            val = double(expr);
            if val <= 0
                fprintf('Row %-2d: %.4f > 0  <-- SYSTEM UNSTABLE (Constant is <= 0)\n', i, val);
                rangos_K = [];
                break;
            else
                fprintf('Row %-2d: %.4f > 0  [Always True]\n', i, val);
                continue;
            end
        end
        
        % Imprimir la desigualdad
        expr_str = char(vpa(expr, 4));
        % Limpiar un poco el string de vpa para que sea más legible
        expr_str = strrep(expr_str, '.0 ', ' ');
        fprintf('Row %-2d: %s > 0\n', i, expr_str);
        
        % Solución Numérica de la Inecuación Evaluando N/D
        [N, D] = numden(expr);
        c_N = double(coeffs(N, K, 'All'));
        c_D = double(coeffs(D, K, 'All'));
        
        % Encontrar raíces reales de numerador y denominador
        r_N = roots(c_N); r_N = real(r_N(abs(imag(r_N)) < 1e-6));
        r_D = roots(c_D); r_D = real(r_D(abs(imag(r_D)) < 1e-6));
        
        crit_pts = unique([r_N; r_D; -Inf; Inf]);
        valid_intervals = [];
        
        % Evaluar el punto medio entre cada raíz crítica
        for k = 1:length(crit_pts)-1
            mid = (crit_pts(k) + crit_pts(k+1))/2;
            if isinf(mid)
                if isinf(crit_pts(k+1)), mid = crit_pts(k) + 10;
                else, mid = crit_pts(k+1) - 10; end
            end
            
            % Comprobar si el intervalo es válido
            if double(subs(expr, K, mid)) > 0
                valid_intervals = [valid_intervals; crit_pts(k), crit_pts(k+1)];
            end
        end
        
        % Intersectar intervalos actuales con los previos
        rangos_K = intersectRanges(rangos_K, valid_intervals);
        
        % Imprimir el resultado de la fila
        if isempty(rangos_K)
            fprintf('   Current Valid Range: NONE (Contradiction detected)\n');
            break; % Ya no hay rango posible
        else
            str_ranges = '';
            for r = 1:size(rangos_K, 1)
                str_ranges = [str_ranges, sprintf('%.4f < K < %.4f   ', rangos_K(r,1), rangos_K(r,2))]; %#ok<AGROW>
            end
            fprintf('   Current Valid Range: %s\n', str_ranges);
        end
        fprintf('   .......................................................................\n');
    end
    
    % --- RESULTADOS GLOBALES ---
    fprintf('\n-> GLOBAL RESULT:\n');
    es_estable = false;
    rangos_K_final = rangos_K;
    
    if isempty(rangos_K_final)
        fprintf('   No stable range of Kp exists for this system.\n');
    else
        fprintf('   Detected stability windows:\n');
        for i = 1:size(rangos_K_final, 1)
            k_inf = rangos_K_final(i,1);
            k_sup = rangos_K_final(i,2);
            
            fprintf('   Interval %d: %.4f  <  K  <  %.4f\n', i, k_inf, k_sup);
        
            if K_test > k_inf && K_test < k_sup
                es_estable = true;
            end
        end
    end
    
    % --- IMPRESIÓN BONITA FINAL ---
    fprintf('\n-----------------------------------------------------------------\n');
    if es_estable
        fprintf('   STATUS: The system IS STABLE for K = %.4f!\n', K_test);
    else
        fprintf('   STATUS: The system IS UNSTABLE for K = %.4f.\n', K_test);
    end
    fprintf('=================================================================\n');
end

%% Helper Function para intersectar rangos
function new_ranges = intersectRanges(rangesA, rangesB)
    new_ranges = [];
    if isempty(rangesA) || isempty(rangesB), return; end
    
    for a = 1:size(rangesA, 1)
        for b = 1:size(rangesB, 1)
            lower = max(rangesA(a, 1), rangesB(b, 1));
            upper = min(rangesA(a, 2), rangesB(b, 2));
            if lower < upper
                new_ranges = [new_ranges; lower, upper];
            end
        end
    end
end