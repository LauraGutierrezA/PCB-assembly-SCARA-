function results = performance_metrics_discrete(Gz, Cz, Ts, varargin)
%PERFORMANCE_METRICS_DISCRETE  Performance metrics for an already-
%   discretized plant Gz (e.g. via bilinear/Tustin, matched-pole-zero,
%   or any other method you used) closed with a discrete controller Cz,
%   both at sample time Ts.
%
%   RESULTS = PERFORMANCE_METRICS_DISCRETE(Gz, Cz, Ts) closes the unity
%   negative feedback loop entirely in discrete time and returns:
%
%       ITAE   - Integral of Time-weighted Absolute Error
%       ISE    - Integral of Squared Error, ISE = int_0^inf e(t)^2 dt
%       %OS    - Percent overshoot
%       ts     - Settling time
%       ICE    - Integral of Control Effort (ISCE = int u^2 dt and
%                IACE = int |u| dt are both returned; see note below)
%       GM/PM  - Gain margin and phase margin of the open loop Cz*Gz
%
%   RESULTS = PERFORMANCE_METRICS_DISCRETE(Gz, Cz, Ts, 'Name', Value, ...)
%
%       'StepAmplitude'      Reference step amplitude (default 1).
%       'SimTime'            Simulation horizon in seconds (default []
%                            = auto, from step()'s own choice).
%       'ICEType'            'squared' (default, ISCE) or 'absolute' (IACE)
%       'SettlingTolerance'  Fractional band for settling time
%                            (default 0.02 for 2%).
%       'Plot'               true/false, default true.
%
%   NOTE ON "ICE": this acronym isn't defined in the course slides
%   (Sessions 1-13), so there's no single confirmed formula for it.
%   Based on standard performance-index literature, "control effort" is
%   reported either as the integral of the squared control signal
%   (energy-like, here ISCE) or the integral of the absolute control
%   signal (IACE). Both are returned; results.ICE picks whichever
%   'ICEType' you select.
%
%   NOTE ON "ISE": Integral of Squared Error, int e(t)^2 dt. This is a
%   standard, unambiguous performance index (Ogata, Seborg) and is
%   returned separately as results.ISE regardless of 'ICEType'.
%
%   Example:
%       Ts = 0.05;
%       Gbw = c2d(Gs, Ts, 'tustin');     % your already-discretized plant
%       Cz  = tf([1.5+1.0*Ts, -1.5],[1 -1], Ts);
%       results = performance_metrics_discrete(Gbw, Cz, Ts);

    p = inputParser;
    addRequired(p, 'Gz', @(x) isa(x,'tf') || isa(x,'zpk') || isa(x,'ss'));
    addRequired(p, 'Cz', @(x) isa(x,'tf') || isa(x,'zpk') || isa(x,'ss'));
    addRequired(p, 'Ts', @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'StepAmplitude', 1, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'SimTime', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
    addParameter(p, 'ICEType', 'squared', @(x) any(strcmpi(x, {'squared','absolute'})));
    addParameter(p, 'SettlingTolerance', 0.02, @(x) isnumeric(x) && isscalar(x) && x > 0 && x < 1);
    addParameter(p, 'Plot', true, @(x) islogical(x) || isnumeric(x));
    parse(p, Gz, Cz, Ts, varargin{:});

    amp     = p.Results.StepAmplitude;
    simTime = p.Results.SimTime;
    iceType = lower(p.Results.ICEType);
    tol     = p.Results.SettlingTolerance;
    doPlot  = logical(p.Results.Plot);

    % ---- Sample-time consistency check ---------------------------------
    tsList = [];
    if isprop(Gz,'Ts') && Gz.Ts ~= 0, tsList(end+1) = Gz.Ts; end %#ok<AGROW>
    if isprop(Cz,'Ts') && Cz.Ts ~= 0, tsList(end+1) = Cz.Ts; end %#ok<AGROW>
    if ~isempty(tsList) && any(abs(tsList - Ts) > 1e-12)
        warning('performance_metrics_discrete:TsMismatch', ...
            'Gz/Cz sample time(s) [%s] do not match the Ts argument (%.6g).', ...
            num2str(tsList), Ts);
    end

    % ---- Build loops -----------------------------------------------------
    L    = Cz * Gz;             % open loop
    T_ry = feedback(L, 1);      % reference -> output
    T_ru = feedback(Cz, Gz);    % reference -> control effort

    isStable = local_check_stability(T_ry);
    if ~isStable
        warning('performance_metrics_discrete:Unstable', ...
            'The closed loop (Cz, Gz) appears unstable or could not be confirmed stable.');
    end

    % ---- Time vector -------------------------------------------------
    if isempty(simTime)
        [y0, t] = step(T_ry);
    else
        t = (0:Ts:simTime)';
        y0 = step(T_ry, t);
    end
    u0 = step(T_ru, t);

    y = amp * y0;
    u = amp * u0;
    e = amp - y;

    % ---- ITAE (rectangular sum, exact for a ZOH-held signal) -----------
    ITAE = sum(t(:) .* abs(e(:))) * Ts;

    % ---- ISE (integral of squared error) --------------------------------
    ISE = sum(e.^2) * Ts;

    % ---- ICE (integral of control effort) --------------------------------
    ISCE = sum(u.^2) * Ts;
    IACE = sum(abs(u)) * Ts;
    ICE = ISCE; if strcmp(iceType,'absolute'), ICE = IACE; end

    % ---- %OS and settling time ------------------------------------------
    if isStable
        info = stepinfo(T_ry, 'SettlingTimeThreshold', tol);
        percentOvershoot = info.Overshoot;
        settlingTime     = info.SettlingTime;
        riseTime         = info.RiseTime;
        peakTime         = info.PeakTime;
    else
        percentOvershoot = NaN; settlingTime = NaN; riseTime = NaN; peakTime = NaN;
    end

    % ---- GM / PM -----------------------------------------------------
    [Gm, Pm, Wcg, Wcp] = margin(L);
    GmdB = 20*log10(Gm);

    % ---- Results ---------------------------------------------------------
    results = struct();
    results.IsStable          = isStable;
    results.ITAE              = ITAE;
    results.ISE               = ISE;
    results.ICE               = ICE;
    results.ICEType           = iceType;
    results.ICE_squared_ISCE  = ISCE;
    results.ICE_absolute_IACE = IACE;
    results.PercentOvershoot  = percentOvershoot;
    results.SettlingTime      = settlingTime;
    results.RiseTime          = riseTime;
    results.PeakTime          = peakTime;
    results.GainMargin_abs    = Gm;
    results.GainMargin_dB     = GmdB;
    results.PhaseMargin_deg   = Pm;
    results.Wcg               = Wcg;
    results.Wcp               = Wcp;
    results.Ts                = Ts;
    results.OpenLoop          = L;
    results.ClosedLoop_ry     = T_ry;
    results.ClosedLoop_ru     = T_ru;
    results.t                 = t;
    results.y                 = y;
    results.u                 = u;
    results.e                 = e;

    % ---- Plots -------------------------------------------------------
    if doPlot
        figure('Name', 'Discrete Performance Metrics Simulation');

        subplot(3,1,1);
        stairs(t, y, 'LineWidth', 1.4); hold on;
        yline(amp, '--k');
        if isStable && ~isnan(settlingTime), xline(settlingTime, '--r'); end
        grid on; xlabel('Time (s)'); ylabel('y[k]');
        title(sprintf('Step response  (%%OS = %.2f%%, ts = %.4g s)', ...
            percentOvershoot, settlingTime));

        subplot(3,1,2);
        stairs(t, u, 'LineWidth', 1.4);
        grid on; xlabel('Time (s)'); ylabel('u[k]');
        title(sprintf('Control effort  (ISCE = %.4g, IACE = %.4g)', ISCE, IACE));

        subplot(3,1,3);
        stairs(t, t.*abs(e), 'LineWidth', 1.2);
        grid on; xlabel('Time (s)'); ylabel('t\cdot|e[k]|');
        title(sprintf('ITAE integrand  (ITAE = %.4g, ISE = %.4g)', ITAE, ISE));

        figure('Name', 'Open-Loop Margins');
        margin(L);
        grid on;
    end
end

% =========================================================================
function ok = local_check_stability(sys)
%LOCAL_CHECK_STABILITY  Wraps isstable() in a try/catch so models with
%   delays or unusual realizations degrade gracefully.
    try
        ok = isstable(sys);
    catch
        warning('performance_metrics_discrete:StabilityCheckFailed', ...
            'isstable() could not evaluate this model; assuming NOT verified stable.');
        ok = false;
    end
end