function [K, zita, wn, tau1, tau2, theta, G_sopdt, G_sopdt_factor, mse, nrmse, fit_pct] = identificarSOPDT(u, y, t)
    % IDENTIFICARSOPDT Método de Stark con detección de u_ss en zona activa,
    % simulación con escalón MANTENIDO, y cálculo de MSE, NRMSE y Fit Percentage.
    arguments
        u (:,1) double
        y (:,1) double
        t (:,1) double
    end
    
    % Asegurar vectores columna
    u = u(:); y = y(:); t = t(:);
    
    % --- 1. DETECCIÓN DEL ESCALÓN DE SUBIDA (t_step) ---
    du = [0; abs(diff(u))];
    idx_step = find(du > 0.1 * max(du), 1, 'first');
    if isempty(idx_step)
        idx_step = 1;
    end
    t_step = t(idx_step);
    
    % Estado inicial de reposo
    y0 = mean(y(1:max(1, idx_step-1)));
    u0 = u(1);
    
    % --- 2. CÁLCULO DE u_ss EN LA PRIMERA ZONA ACTIVA ---
    % Buscar si la entrada se apaga (cae) más adelante
    du_real = [0; diff(u)];
    idx_caida_u = find(du_real(idx_step+1:end) < -0.1 * max(du), 1, 'first');
    
    if ~isempty(idx_caida_u)
        % Si cae, la zona estable activa llega hasta antes de la caída
        idx_fin_u_activo = idx_step + idx_caida_u - 1;
    else
        % Si no cae, la zona activa llega hasta el final del dataset
        idx_fin_u_activo = length(u);
    end
    
    % Promedio de u en el tramo final de la zona activa (último 20% de la zona encendida)
    num_ss_u = max(3, round((idx_fin_u_activo - idx_step) * 0.2));
    idx_ss_u_inicio = max(idx_step + 1, idx_fin_u_activo - num_ss_u);
    u_ss = mean(u(idx_ss_u_inicio:idx_fin_u_activo));
    
    % --- 3. CÁLCULO DE y_ss AL FINAL DEL DATASET ---
    % Estado estable de la salida en el último 10%
    num_ss_y = max(5, round(length(y) * 0.1));
    y_ss = mean(y(end-num_ss_y:end));
    
    % --- 4. GANANCIA Y VALORES OBJETIVO ---
    delta_y = y_ss - y0;
    delta_u = u_ss - u0;
    
    % Protección por si delta_u es numéricamente muy pequeño
    if abs(delta_u) < 1e-5
        delta_u = max(u) - min(u);
    end
    
    K = delta_y / delta_u;
    
    val_t1 = y0 + 0.15 * delta_y; % 15%
    val_t2 = y0 + 0.45 * delta_y; % 45%
    val_t3 = y0 + 0.75 * delta_y; % 75%
    
    % --- 5. EXTRACCIÓN ROBUSTA DE LA CURVA ASCENDENTE ---
    y_subida = y(idx_step:end);
    t_subida = t(idx_step:end);
    
    % Forzar curva estrictamente creciente (elimina retrocesos por ruido)
    idx_mono = true(size(y_subida));
    max_val = y_subida(1);
    for i = 2:length(y_subida)
        if y_subida(i) <= max_val
            idx_mono(i) = false;
        else
            max_val = y_subida(i);
        end
    end
    y_clean = y_subida(idx_mono);
    t_clean = t_subida(idx_mono);
    
    % Interpolación lineal/pchip garantizada para los 3 puntos
    t1_abs = interp1(y_clean, t_clean, val_t1, 'pchip', 'extrap');
    t2_abs = interp1(y_clean, t_clean, val_t2, 'pchip', 'extrap');
    t3_abs = interp1(y_clean, t_clean, val_t3, 'pchip', 'extrap');
    
    % Tiempos relativos al momento del escalón
    t1 = t1_abs - t_step;
    t2 = t2_abs - t_step;
    t3 = t3_abs - t_step;
    
    % --- 6. CÁLCULO DE x Y VERIFICACIONES DE STARK ---
    den_x = (t3 - t1);
    if den_x <= 0
        error('Error en el vector de datos: t3 debe ser mayor que t1.');
    end
    x = (t2 - t1) / den_x;
    
    % Verificación de límites válidos para la aproximación de Stark
    if x <= 0.356
        warning('El valor de x (%.4f) está por debajo del límite de Stark (0.356). Se ajustó levemente para continuar.', x);
        x = 0.357; 
    end
    
    zita = (0.0805 - 5.547 * (0.475 - x)^2) / (x - 0.356);
    if zita < 1
        error('No se puede utilizar este método: El factor de amortiguamiento calculado (zita = %.4f) es menor que 1 (el sistema no es sobreamortiguado).', zita);
    end
    
    % --- 7. PARÁMETROS F2, F3, wn Y THETA ---
    F2 = 2.6 * zita - 0.6;
    F3 = 0.922 * (1.66^zita);
    wn = F2 / (t3 - t1);
    theta = t2 - (F3 / wn);
    if theta < 0
        theta = 0;
    end
    
    % --- 8. FACTORIZACIÓN DE TAU1 Y TAU2 ---
    disc = sqrt(zita^2 - 1);
    tau1 = (zita + disc) / wn;
    tau2 = (zita - disc) / wn;
    
    s = tf('s');
    G_sopdt = (K * (wn^2) * exp(-theta * s)) / (s^2 + 2 * zita * wn * s + wn^2);
    G_sopdt_factor = (K * exp(-theta * s)) / ((tau1 * s + 1) * (tau2 * s + 1));
    
    % --- 9. SIMULACIÓN Y CÁLCULO DE MÉTRICAS DE ERROR (MSE, NRMSE, FIT %) ---
    t_sim = t - t(1);
    
    % Generación sintética del escalón mantenido (sin apagado)
    u_sim = zeros(size(u));
    u_sim(idx_step:end) = delta_u;
    
    y_sim = lsim(G_sopdt_factor, u_sim, t_sim) + y0;
    
    % Cálculos de error
    mse = mean((y - y_sim).^2);
    rmse = sqrt(mse);
    
    % NRMSE normalizado con el rango de la salida real
    rango_y = max(y) - min(y);
    if rango_y < 1e-5, rango_y = 1; end % Protección división por cero
    nrmse = rmse / rango_y;
    
    % Fit Percentage (% de Ajuste estilo MATLAB toolbox)
    norm_res = norm(y - y_sim);
    norm_mean = norm(y - mean(y));
    if norm_mean < 1e-5, norm_mean = 1; end
    fit_pct = (1 - norm_res / norm_mean) * 100;
    
    % --- IMPRESIÓN ---
    fprintf('\n=== SOPDT Identification  ===\n');
    fprintf('Tiempos detectados: t1=%.4fs, t2=%.4fs, t3=%.4fs | x=%.4f\n', t1, t2, t3, x);
    fprintf('Ganancia (K):           %.4f\n', K);
    fprintf('Amortiguamiento (zita): %.4f\n', zita);
    fprintf('Frecuencia nat. (wn):   %.4f rad/s\n', wn);
    fprintf('Tau 1:                  %.4f s\n', tau1);
    fprintf('Tau 2:                  %.4f s\n', tau2);
    fprintf('Tiempo muerto (theta):  %.4f s\n', theta);
    fprintf('MSE del ajuste:         %.6f\n', mse);
    fprintf('NRMSE:                  %.4f (%.2f%% del rango)\n', nrmse, nrmse * 100);
    fprintf('Fit Percentage:         %.2f%%\n', fit_pct);
    fprintf('--------------------------------------------------------\n');
    
    % --- GRAFICACIÓN ---
    figure;
    plot(t, y, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Datos Reales (y)'); hold on;
    plot(t, y_sim, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Modelo SOPDT (Escalón Mantenido)');
    plot(t1_abs, val_t1, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '15% (t_1)');
    plot(t2_abs, val_t2, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '45% (t_2)');
    plot(t3_abs, val_t3, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '75% (t_3)');
    grid on;
    xlabel('Time (s)'); ylabel('Output (%)');
    title(sprintf(' SOPDT Identification| K = %.2f, \\zeta = %.2f, \\omega_n = %.2f | Fit = %.1f%%', ...
        K, zita, wn, fit_pct));
    legend('Location', 'best');
end