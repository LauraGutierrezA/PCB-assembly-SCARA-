function [rangos_K_final, es_estable] = controllerJury(G_disc, K_test)
    % CONTROLLERJURY Jury's Stability Criterion via exact symbolic root-finding
    % (no numerical sweep). Mirrors the approach used in controllerRouth:
    % every inequality is turned into a polynomial in K, its critical points
    % (roots) are found exactly, and validity is tested once per resulting
    % sub-interval. Always reports the TRUE stability window (including
    % negative K, if any) - no K > 0 clamping is applied.
    %
    % Inputs:
    %   G_disc : Discrete transfer function
    %   K_test : Value of K to evaluate (Default: 1)

    arguments
        G_disc
        K_test (1,1) double = 1
    end

    fprintf('\n=== JURY STABILITY ANALYSIS (EXACT ROOT-FINDING) ===\n');

    % 1. Prepare polynomials
    if hasdelay(G_disc), G_disc = delay2z(G_disc); end
    [num, den] = tfdata(G_disc, 'v');

    n_den = length(den); n_num = length(num);
    if n_den > n_num, num = [zeros(1, n_den - n_num), num];
    elseif n_num > n_den, den = [zeros(1, n_num - n_den), den]; end
    n = length(den) - 1;

    fprintf('1. Characteristic equation Q(z) = 1 + G(z)H(z) = 0\n');
    fprintf('   Detected system degree (n) = %d\n\n', n);

    % --- STEP 2: Q(1) > 0 (informational, sign of leading coeff handled below) ---
    Q1_K = polyval(num, 1); Q1_ind = polyval(den, 1);
    lim_Q1 = -Q1_ind / Q1_K;
    fprintf('2. Verification Q(1) > 0\n');
    fprintf('   Q(1) = %.4f*K + %.4f > 0\n', Q1_K, Q1_ind);
    if Q1_K > 0
        fprintf('   Raw root (before sign convention): K = %.4f\n\n', lim_Q1);
    else
        fprintf('   Raw root (before sign convention): K = %.4f\n\n', lim_Q1);
    end

    % --- STEP 3: (-1)^n * Q(-1) > 0 ---
    Qneg1_K = polyval(num, -1); Qneg1_ind = polyval(den, -1);
    lim_Qneg1 = -Qneg1_ind / Qneg1_K;
    fprintf('3. Verification (-1)^n * Q(-1) > 0\n');
    fprintf('   Q(-1) = %.4f*K + %.4f\n', Qneg1_K, Qneg1_ind);
    fprintf('   Raw root: K = %.4f\n\n', lim_Qneg1);

    % --- STEP 4: |a0| < |an| ---
    an_K = num(1); an_ind = den(1);
    a0_K = num(end); a0_ind = den(end);
    fprintf('4. Verification |a0| < |an|\n');
    fprintf('   |%.4f*K + %.4f| < |%.4f*K + %.4f|\n', a0_K, a0_ind, an_K, an_ind);
    fprintf('   (Exact roots resolved analytically below)\n\n');

    % --- SYMBOLIC JURY ARRAY ---
    fprintf('5. Building exact symbolic Jury array...\n\n');
    syms K real
    a_sym = den + K * num;

    tabla_sym = cell(max(2*n - 3, 2), 1);
    tabla_sym{1} = fliplr(a_sym);
    tabla_sym{2} = a_sym;
    r1_sym = tabla_sym{1};

    for j = 1:(n - 2)
        r2_sym = fliplr(r1_sym);
        m = length(r1_sym) - 1;
        next_r_sym = sym(zeros(1, m));
        for k = 1:m
            next_r_sym(k) = expand(r1_sym(1)*r1_sym(k) - r2_sym(1)*r2_sym(k));
        end

        fila_impar = 2*j + 1;
        tabla_sym{fila_impar} = next_r_sym;
        if fila_impar < 2*n - 3
            tabla_sym{fila_impar + 1} = fliplr(next_r_sym);
        end
        r1_sym = next_r_sym;
    end

    % --- Diagnostic: which row (if any) fails for the given K_test ---
    a_test = den + K_test * num;
    if a_test(1) < 0, a_test = -a_test; end
    r1_test = fliplr(a_test);
    fila_falla = -1;
    for j = 1:(n - 2)
        r2_test = fliplr(r1_test);
        m = length(r1_test) - 1;
        next_r_test = zeros(1, m);
        for k = 1:m
            next_r_test(k) = r1_test(1)*r1_test(k) - r2_test(1)*r2_test(k);
        end
        if fila_falla == -1 && abs(next_r_test(1)) <= abs(next_r_test(end))
            fila_falla = 2*j + 1;
        end
        r1_test = next_r_test;
    end

    % --- FILTERED SYMBOLIC ARRAY PRINTING ---
    fprintf('=== SYMBOLIC JURY ARRAY (Visualization) ===\n');
    filas_a_imprimir = unique([1, 2, fila_falla, fila_falla+1, 2*n-4, 2*n-3]);
    filas_a_imprimir(filas_a_imprimir <= 0 | filas_a_imprimir > 2*n-3) = [];

    for i = 1:length(filas_a_imprimir)
        idx = filas_a_imprimir(i);
        es_falla = (idx == fila_falla);
        sym_row = tabla_sym{idx};

        if length(sym_row) >= 3
            c1 = char(vpa(sym_row(1), 4));
            c2 = char(vpa(sym_row(2), 4));
            cend = char(vpa(sym_row(end), 4));
            str_vals = sprintf('[ %s ]   [ %s ]  ...  [ %s ]', c1, c2, cend);
        else
            str_vals = sprintf('[ %s ]   [ %s ]', char(vpa(sym_row(1), 4)), char(vpa(sym_row(end), 4)));
        end

        if es_falla
            fprintf('Row  %-2d: %s  <-- ¡NUMERICAL BREAKDOWN DETECTED HERE!\n', idx, str_vals);
        else
            fprintf('Row  %-2d: %s\n', idx, str_vals);
        end

        if i < length(filas_a_imprimir) && (filas_a_imprimir(i+1) - idx > 1)
            fprintf('\n   .......................................................................\n\n');
        end
    end

    % --- EXACT CRITICAL POINTS (replaces the numerical sweep) ---
    fprintf('\n6. Solving all inequalities exactly for critical K values...\n\n');

    crit_pts = [];

    % (a) Leading coefficient sign change: an(K) = 0
    an_expr = den(1) + K*num(1);
    crit_pts = [crit_pts; local_real_roots(an_expr, K)];

    % (b) Q(1) = 0
    Q1_expr = Q1_K*K + Q1_ind;
    crit_pts = [crit_pts; local_real_roots(Q1_expr, K)];

    % (c) Q(-1) = 0
    Qneg1_expr = Qneg1_K*K + Qneg1_ind;
    crit_pts = [crit_pts; local_real_roots(Qneg1_expr, K)];

    % (d) |a0| = |an|  ->  an^2 - a0^2 = 0
    a0_expr = den(end) + K*num(end);
    AnA0_expr = an_expr^2 - a0_expr^2;
    crit_pts = [crit_pts; local_real_roots(AnA0_expr, K)];

    % (e) Each recursive row: |row(1)| = |row(end)|  ->  row(1)^2 - row(end)^2 = 0
    for j = 1:(n - 2)
        row = tabla_sym{2*j + 1};
        row_expr = expand(row(1)^2 - row(end)^2);
        crit_pts = [crit_pts; local_real_roots(row_expr, K)];
    end

    crit_pts = sort(unique([-Inf; Inf; crit_pts]));

    % --- Test validity once per sub-interval (no grid, just critical points) ---
    n_int = length(crit_pts) - 1;
    valid_flags = false(n_int, 1);
    for k = 1:n_int
        lo = crit_pts(k); hi = crit_pts(k+1);
        if (hi - lo) < 1e-9, continue; end
        if isinf(lo) && isinf(hi)
            mid = 0;
        elseif isinf(lo)
            mid = hi - 10;
        elseif isinf(hi)
            mid = lo + 10;
        else
            mid = (lo + hi) / 2;
        end
        valid_flags(k) = local_jury_valid(mid, num, den, n);
    end

    % --- Merge adjacent valid sub-intervals into final ranges ---
    rangos_K = [];
    k = 1;
    while k <= n_int
        if valid_flags(k)
            lo = crit_pts(k);
            while k <= n_int && valid_flags(k)
                hi = crit_pts(k+1);
                k = k + 1;
            end
            rangos_K = [rangos_K; lo, hi]; %#ok<AGROW>
        else
            k = k + 1;
        end
    end

    % --- GLOBAL RESULTS (TRUE stability window, no K > 0 clamping) ---
    fprintf('-> GLOBAL RESULT:\n');
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

    % --- FINAL CLEAN OUTPUT ---
    fprintf('\n-----------------------------------------------------------------\n');
    if es_estable
        fprintf('   STATUS: The system IS STABLE for K = %.4f!\n', K_test);
        if size(rangos_K_final, 1) == 1
            fprintf('   Recommended gain range:  %.4f < K < %.4f\n', rangos_K_final(1,1), rangos_K_final(1,2));
        else
            fprintf('   Recommended gain ranges:\n');
            for i = 1:size(rangos_K_final, 1)
                fprintf('     -> Interval %d: %.4f < K < %.4f\n', i, rangos_K_final(i,1), rangos_K_final(i,2));
            end
        end
    else
        fprintf('   STATUS: The system IS UNSTABLE for K = %.4f.\n', K_test);
        if ~isempty(rangos_K_final)
            fprintf('   The allowed stable range is:\n');
            for i = 1:size(rangos_K_final, 1)
                fprintf('     -> Interval %d: %.4f < K < %.4f\n', i, rangos_K_final(i,1), rangos_K_final(i,2));
            end
        end
    end
    fprintf('=================================================================\n');
end

%% Helper: exact real roots of a polynomial expression in K
function r = local_real_roots(expr, K)
    expr = expand(expr);
    if isempty(symvar(expr))
        r = [];
        return;
    end
    old_d = digits(100);         % suficiente para el rango dinámico de ~1e50
    sols = vpasolve(expr == 0, K);   % conserva precisión arbitraria, no double()
    digits(old_d);
    sols = double(sols);
    r = real(sols(abs(imag(sols)) < 1e-6));
end

%% Helper: exact re-implementation of the Jury validity test at one K value
% (Same logic as the original per-K check, just evaluated at analytically
% chosen points instead of a 100,000-point sweep.)
function valid = local_jury_valid(Kp, num, den, n)
    a = den + Kp * num;
    if a(1) < 0, a = -a; end
    if abs(a(1)) < 1e-10 * max(abs(a))
        valid = false; return;
    end
    if polyval(a, 1) <= 0 || (((-1)^n) * polyval(a, -1)) <= 0 || abs(a(end)) >= abs(a(1))
        valid = false; return;
    end

    r1 = fliplr(a);
    for j = 1:(n - 2)
        r2 = fliplr(r1);
        m = length(r1) - 1;
        next_r = zeros(1, m);
        for k = 1:m
            next_r(k) = r1(1)*r1(k) - r2(1)*r2(k);
        end
        r1 = next_r;
        if abs(r1(1)) <= abs(r1(end))
            valid = false; return;
        end
    end
    valid = true;
end