function [K, tau, theta, G_fopdt, mse] = identificarFOPDTLSIM(u, y, t)
    % IDENTIFICARFOPDT Identificación FOPDT con sincronización de entrada.
    % Calcula u_ss en la primera zona estable activa (u != 0) e y_ss al final del dataset.
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
    % Buscamos si la entrada se apaga más adelante
    du_real = [0; diff(u)];
    idx_caida_u = find(du_real(idx_step+1:end) < -0.1 * max(du), 1, 'first');
    
    if ~isempty(idx_caida_u)
        % Si la entrada vuelve a caer, el primer estado estable de u está antes de esa caída
        idx_fin_u_activo = idx_step + idx_caida_u - 1;
    else
        % Si la entrada no cae, el estado activo llega hasta el final
        idx_fin_u_activo = length(u);
    end
    
    % Promedio de u en la zona alta activa (últimas muestras del primer escalón estable)
    num_ss_u = max(3, round((idx_fin_u_activo - idx_step) * 0.2));
    idx_ss_u_inicio = max(idx_step + 1, idx_fin_u_activo - num_ss_u);
    u_ss = mean(u(idx_ss_u_inicio:idx_fin_u_activo));

    % --- 3. CÁLCULO DE y_ss AL FINAL DEL DATASET ---
    % Se evalúa en el último 10% de la salida y(t) donde alcanza su régimen permanente
    num_ss_y = max(5, round(length(y) * 0.1));
    y_ss = mean(y(end-num_ss_y:end));
    
    % --- 4. GANANCIA (K) ---
    delta_y = y_ss - y0;
    delta_u = u_ss - u0;
    
    if abs(delta_u) < 1e-5
        % Respaldo por seguridad si u0 == u_ss
        delta_u = max(u) - min(u);
    end
    
    K = delta_y / delta_u;
    
    % --- 5. TIEMPOS t1 (28.3%) Y t2 (63.2%) ---
    val_t1 = y0 + 0.283 * delta_y;
    val_t2 = y0 + 0.632 * delta_y;
    
    % Tramo de subida de la salida y(t)
    y_subida = y(idx_step:end);
    t_subida = t(idx_step:end);
    
    % Eliminar duplicados continuos en y manteniendo el tiempo ordenado
    [y_clean, idx_clean] = unique(y_subida, 'stable');
    t_clean = t_subida(idx_clean);
    
    % Interpolación lineal
    t1_abs = interp1(y_clean, t_clean, val_t1, 'linear', 'extrap');
    t2_abs = interp1(y_clean, t_clean, val_t2, 'linear', 'extrap');
    
    % Tiempos relativos AL MOMENTO DEL ESCALÓN
    t1 = t1_abs - t_step;
    t2 = t2_abs - t_step;
    
    % --- 6. PARÁMETROS DE SMITH ---
    tau = 1.5 * (t2 - t1);
    theta = t2 - tau;
    
    % Parches de seguridad física
    if tau <= 0, tau = max(1e-3, (t2_abs - t_step) * 0.6); end
    if theta < 0, theta = 0; end
    
    % --- 7. FUNCIÓN DE TRANSFERENCIA Y SIMULACIÓN ---
    s = tf('s');
    G_fopdt = (K * exp(-theta * s)) / (tau * s + 1);
    
    % Tiempo normalizado para simulación
    t_sim = t - t(1);
    u_sim = u - u0;
    
    % Simulación de la respuesta completa
    y_sim = lsim(G_fopdt, u_sim, t_sim) + y0;
    
    % --- 8. CÁLCULO DE MSE Y GRÁFICA ---
    mse = mean((y - y_sim).^2);
    
    fprintf('\n=== Ajuste FOPDT Sincronizado ===\n');
    fprintf('Ganancia (K):            %.4f\n', K);
    fprintf('Constante de tiempo (t): %.4f s\n', tau);
    fprintf('Tiempo muerto (theta):   %.4f s\n', theta);
    fprintf('MSE del ajuste:          %.6f\n', mse);
    G_fopdt
    fprintf('---------------------------------\n');
    
    % Graficar
    figure;
    plot(t, y, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Datos Reales (y)'); hold on;
    plot(t, y_sim, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Modelo FOPDT');
    
    % Dibujar puntos de prueba Smith
    plot(t1_abs, val_t1, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '28.3% (t_1)');
    plot(t2_abs, val_t2, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'DisplayName', '63.2% (t_2)');
    
    grid on;
    xlabel('Tiempo (s)'); ylabel('Salida');
    title(sprintf('Identificación Smith | K = %.3f, \\tau = %.3fs, \\theta = %.3fs | MSE = %.4f', K, tau, theta, mse));
    legend('Location', 'best');
end