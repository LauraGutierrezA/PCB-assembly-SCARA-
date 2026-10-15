function [K, tau, theta, G_fopdt, mse, nrmse, fit_pct] = identificarFOPDT(u, y, t)
    % IDENTIFICARFOPDT Identificación FOPDT con simulación de escalón mantenido,
    % cálculo de MSE, NRMSE y Fit Percentage.
    arguments
        u (:,1) double
        y (:,1) double
        t (:,1) double
    end
    
    % Asegurar vectores columna
    u = u(:); y = y(:); t = t(:);
    
    % --- 1. DETECTAR EL MOMENTO DEL ESCALÓN DE SUBIDA ---
    du = [0; abs(diff(u))];
    idx_step = find(du > 0.1 * max(du), 1, 'first');
    if isempty(idx_step)
        idx_step = 1;
    end
    t_step = t(idx_step);
    
    % Estado inicial de reposo
    y0 = mean(y(1:max(1, idx_step-1)));
    u0 = u(1);
    
    % --- 2. CÁLCULO DE u_ss EN EL PRIMER ESTABLE DISTINTO DE CERO ---
    du_real = [0; diff(u)];
    idx_caida_u = find(du_real(idx_step+1:end) < -0.1 * max(du), 1, 'first');
    
    if ~isempty(idx_caida_u)
        idx_fin_u_activo = idx_step + idx_caida_u - 1;
    else
        idx_fin_u_activo = length(u);
    end
    
    num_ss_u = max(3, round((idx_fin_u_activo - idx_step) * 0.2));
    idx_ss_u_inicio = max(idx_step + 1, idx_fin_u_activo - num_ss_u);
    u_ss = mean(u(idx_ss_u_inicio:idx_fin_u_activo));
    
    % --- 3. CÁLCULO DE y_ss AL FINAL DEL DATASET ---
    num_ss_y = max(5, round(length(y) * 0.1));
    y_ss = mean(y(end-num_ss_y:end));
    
    % --- 4. GANANCIA (K) ---
    delta_y = y_ss - y0;
    delta_u = u_ss - u0;
    
    if abs(delta_u) < 1e-5
        delta_u = max(u) - min(u);
    end
    
    K = delta_y / delta_u;
    
    % --- 5. TIEMPOS t1 (28.3%) Y t2 (63.2%) ---
    val_t1 = y0 + 0.283 * delta_y;
    val_t2 = y0 + 0.632 * delta_y;
    
    y_subida = y(idx_step:end);
    t_subida = t(idx_step:end);
    
    [y_clean, idx_clean] = unique(y_subida, 'stable');
    t_clean = t_subida(idx_clean);
    
    t1_abs = interp1(y_clean, t_clean, val_t1, 'linear', 'extrap');
    t2_abs = interp1(y_clean, t_clean, val_t2, 'linear', 'extrap');
    
    t1 = t1_abs - t_step;
    t2 = t2_abs - t_step;
    
    % --- 6. PARÁMETROS DE SMITH ---
    tau = 1.5 * (t2 - t1);
    theta = t2 - tau;
    
    if tau <= 0, tau = max(1e-3, (t2_abs - t_step) * 0.6); end
    if theta < 0, theta = 0; end
    
    % --- 7. FUNCIÓN DE TRANSFERENCIA Y SIMULACIÓN ESCALÓN MANTENIDO ---
    s = tf('s');
    G_fopdt = (K * exp(-theta * s)) / (tau * s + 1);
    
    t_sim = t - t(1);
    
    % Entrada escalón mantenido sintético
    u_sim = zeros(size(u));
    u_sim(idx_step:end) = delta_u;
    
    % Simulación de respuesta
    y_sim = lsim(G_fopdt, u_sim, t_sim) + y0;
    
    % --- 8. CÁLCULO DE MÉTRICAS DE ERROR (MSE, NRMSE, FIT %) ---
    mse = mean((y - y_sim).^2);
    rmse = sqrt(mse);
    
    % NRMSE normalizado con el rango de la salida real
    rango_y = max(y) - min(y);
    if rango_y < 1e-5, rango_y = 1; end % Protección contra división por cero
    nrmse = rmse / rango_y;
    
    % Fit Percentage (% de Ajuste estilo MATLAB toolbox)
    norm_res = norm(y - y_sim);
    norm_mean = norm(y - mean(y));
    if norm_mean < 1e-5, norm_mean = 1; end
    fit_pct = (1 - norm_res / norm_mean) * 100;
    
    % --- IMPRESIÓN ---
    fprintf('\n=== Ajuste FOPDT Sincronizado ===\n');
    fprintf('Ganancia (K):            %.4f\n', K);
    fprintf('Constante de tiempo (t): %.4f s\n', tau);
    fprintf('Tiempo muerto (theta):   %.4f s\n', theta);
    fprintf('MSE del ajuste:          %.6f\n', mse);
    fprintf('NRMSE:                   %.4f (%.2f%% del rango)\n', nrmse, nrmse * 100);
    fprintf('Fit Percentage:          %.2f%%\n', fit_pct);
    G_fopdt
    fprintf('---------------------------------\n');
    
    % --- GRAFICACIÓN ---
    figure;
    plot(t, y, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Datos Reales (y)'); hold on;
    plot(t, y_sim, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Modelo FOPDT');
    
    plot(t1_abs, val_t1, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '28.3% (t_1)');
    plot(t2_abs, val_t2, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '63.2% (t_2)');
    
    grid on;
    xlabel('Time (s)'); ylabel('Output (%)');
    title(sprintf('FOPDT Identification | K = %.3f, \\tau = %.3fs, \\theta = %.3fs | Fit = %.1f%%', ...
        K, tau, theta, fit_pct));
    legend('Location', 'best');
end