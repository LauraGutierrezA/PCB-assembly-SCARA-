function [par_fit, G_fit, fit_pct] = ajustarParametrico(G_ref, Ra, La, p0, opts)
    % AJUSTARPARAMETRICO Tunes beta, J and Kv of the DC motor parametric model
    % so its step response matches a delay-free non-parametric TF.
    % G_ref and G_fit in SI: Omega_motor(s)/Va(s) [rad/s / V].
    arguments
        G_ref {mustBeA(G_ref, {'tf','zpk','ss'})}
        Ra    (1,1) double {mustBePositive}     % fixed [Ohm]
        La    (1,1) double {mustBePositive}     % fixed [H]
        p0    (1,3) double {mustBePositive}     % [beta J Kv] initial guess
        opts.Va   (1,1) double = 12             % step amplitude for the plot [V]
        opts.N    (1,1) double = 50             % gear ratio (plot in RPM at output)
        opts.Name (1,:) char   = 'G'            % label for titles and legend
    end

    % === 1. VALIDATION ===
    if totaldelay(G_ref) > 0
        error('%s has a delay. Remove it before fitting.', opts.Name);
    end

    % === 2. PARAMETRIC MODEL (Ra, La fixed) ===
    Gpar = @(p) tf(p(3)/(p(2)*La), ...
        [1, (p(2)*Ra + p(1)*La)/(p(2)*La), (Ra*p(1) + p(3)^2)/(p(2)*La)]);

    % === 3. TIME VECTOR AND REFERENCE ===
    [~, tt] = step(G_ref);
    t     = linspace(0, tt(end), 1000)';
    y_ref = step(G_ref, t);

    % === 4. FIT (log scale: p = p0.*exp(x), always positive) ===
    lsq_opts = optimoptions('lsqnonlin', 'Display', 'off', ...
        'FunctionTolerance', 1e-12, 'StepTolerance', 1e-12, ...
        'MaxFunctionEvaluations', 3000, 'MaxIterations', 1000);
    cost = @(x) step(Gpar(p0.*exp(x)), t) - y_ref;
    x    = lsqnonlin(cost, zeros(1,3), [], [], lsq_opts);
    p    = p0.*exp(x);

    par_fit = struct('beta', p(1), 'J', p(2), 'Kv', p(3));
    G_fit   = Gpar(p);

    % === 5. METRICS ===
    y_fit   = step(G_fit, t);
    fit_pct = 100*(1 - norm(y_fit - y_ref)/norm(y_ref - mean(y_ref)));

    % === 6. PRINT ===
    names = {'beta', 'J', 'Kv'};
    fprintf('\n=== PARAMETRIC FIT: %s ===\n', opts.Name);
    fprintf('%-6s %12s %12s %10s\n', 'Param', 'Initial', 'Fitted', 'Change');
    for i = 1:3
        fprintf('%-6s %12.4e %12.4e %+9.1f%%\n', names{i}, p0(i), p(i), 100*(p(i)/p0(i) - 1));
    end
    fprintf('Fit Percentage: %.2f%%\n', fit_pct);
    fprintf('\n=== FITTED TF: %s [rad/s / V] ===\n', opts.Name);
    G_fit

    % === 7. PLOT (RPM at gearbox output) ===
    k_rpm = 60/(2*pi*opts.N);
    figure;
    step(opts.Va*k_rpm*G_ref, opts.Va*k_rpm*G_fit, '--'); grid on;
    legend(opts.Name, 'Parametric fit', 'Location', 'southeast');
    title(sprintf('Step %g V | %s vs Parametric | Fit = %.1f%%', opts.Va, opts.Name, fit_pct));
    ylabel('\omega_{out} [RPM]');
end