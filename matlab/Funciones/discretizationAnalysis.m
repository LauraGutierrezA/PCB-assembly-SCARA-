function [Gd, rango_K_routh, rango_K_jury, Ts] = discretizationAnalysis(G, opts)
    % DISCRETIZATIONANALYSIS Chooses Ts combining:
    %  (A) plant criteria (bandwidth, equivalent tau, settling time) evaluated
    %      on G without integrators (same dynamics, finite static gain), and
    %  (B) closed-loop requirement: at least Nmin samples within the desired
    %      closed-loop settling time ts_cl (optional).
    % A user-defined Ts can be given instead; it is checked against (A).
    % Then discretizes G (with integrators) with ZOH, plots continuous vs
    % discrete, and runs Routh/Jury stability on the discrete model.
    % Output label is deduced: integrator -> position [rad], else velocity [rad/s].
    arguments
        G {mustBeA(G, {'tf','zpk','ss'})}
        opts.Ts    (1,1) double = NaN                      % user-defined Ts [s] (optional)
        opts.ts_cl (1,1) double = NaN                      % desired closed-loop settling time [s]
        opts.Nmin  (1,1) double {mustBePositive} = 10      % min samples within ts_cl
        opts.Name  (1,:) char = 'G'                        % label for titles
    end

    % === 1. SAMPLING TIME - (A) PLANT CRITERIA ===
    % Integrators (poles at s = 0) make the static gain infinite, so the
    % criteria are evaluated on G without them.
    nInt = sum(abs(pole(G)) < 1e-9);
    if nInt > 0
        G_ts = minreal(G * tf('s')^nInt);
    else
        G_ts = G;
    end

    [~, ts_min_bw,  ts_max_bw]  = evalc('identificarSamplingTime(G_ts)');          % Bandwidth
    [~, ts_min_tau, ts_max_tau] = evalc('identificarSamplingTimeTauEq(G_ts)');     % Equivalent tau
    [~, ts_min_st,  ts_max_st]  = evalc('identificarSamplingTimeSettling(G_ts)');  % Settling time

    Ts_low  = max([ts_min_bw, ts_min_tau, ts_min_st]);
    Ts_high = min([ts_max_bw, ts_max_tau, ts_max_st]);
    if Ts_low > Ts_high
        warning('No common range: max(Ts_min) = %.4g s > min(Ts_max) = %.4g s', Ts_low, Ts_high);
    end
    Ts_A = Ts_low;

    % === 2. SAMPLING TIME - (B) CLOSED-LOOP REQUIREMENT ===
    if isnan(opts.ts_cl)
        Ts_B = Inf;                         % not specified
    else
        Ts_B = opts.ts_cl / opts.Nmin;
    end

    % === 3. CHOSEN Ts ===
    if ~isnan(opts.Ts)
        Ts = opts.Ts;  source = 'user-defined';
        if Ts < Ts_low || Ts > Ts_high
            warning('User Ts = %.4g s is outside the plant range [%.4g, %.4g] s', Ts, Ts_low, Ts_high);
        end
        if Ts > Ts_B
            warning('User Ts = %.4g s gives fewer than %g samples within ts_cl = %.3g s', Ts, opts.Nmin, opts.ts_cl);
        end
    elseif Ts_B < Ts_A
        Ts = Ts_B;  source = sprintf('closed-loop (ts_cl/%g)', opts.Nmin);
    else
        Ts = Ts_A;  source = 'plant criteria';
    end

    % === 4. DISCRETIZATION ===
    Gd = c2d(G, Ts, 'zoh');

    % === 5. SIMULATION ===
    if nInt > 0
        ylab = '$\theta$ [rad]';
    else
        ylab = '$\omega$ [rad/s]';
    end

    figure;
    t_end = 3*stepinfo(G_ts).SettlingTime;    % a few plant time constants
    step(G, Gd, '--', t_end); grid on;
    legend('Continuous', sprintf('Discrete (Ts = %.3g s)', Ts), 'Location', 'northwest');
    title(sprintf('Discretization %s', opts.Name));
    xlabel('Time');
    ylabel(ylab, 'Interpreter', 'latex');

    % === 6. STABILITY (silent, discrete model) ===
    [~, rango_K_routh] = evalc('controllerRouth(Gd)');
    [~, rango_K_jury]  = evalc('controllerJury(Gd)');

    % === 7. PRINT ===
    fprintf('\n=== DISCRETIZATION ANALYSIS: %s ===\n', opts.Name);
    if nInt > 0
        fprintf('(%d integrator(s) removed to compute Ts)\n', nInt);
    end
    fprintf('Ts range (A) plant = %.4g - %.4g s\n', Ts_low, Ts_high);
    if ~isnan(opts.ts_cl)
        fprintf('Ts (B) closed-loop = %.4g s  (ts_cl = %.3g s, Nmin = %g)\n', Ts_B, opts.ts_cl, opts.Nmin);
    end
    fprintf('Ts used            = %.4g s  [%s]\n', Ts, source);
    fprintf('Routh : %s\n', formatRanges(rango_K_routh));
    fprintf('Jury  : %s\n', formatRanges(rango_K_jury));
end

%% Helper: ranges matrix [lo hi; ...] -> text
function str = formatRanges(r)
    if isempty(r)
        str = 'no stable range';
        return
    end
    parts = arrayfun(@(i) sprintf('%.4f < K < %.4f', r(i,1), r(i,2)), ...
        1:size(r,1), 'UniformOutput', false);
    str = strjoin(parts, '  U  ');
end