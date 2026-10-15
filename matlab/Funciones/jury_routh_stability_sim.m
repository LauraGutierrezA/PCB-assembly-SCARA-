function results = jury_routh_stability_sim(G, routh_range, jury_range, varargin)
%JURY_ROUTH_STABILITY_SIM Closed-loop step responses at Jury/Routh boundary K values.
%
%   results = jury_routh_stability_sim(G, routh_range, jury_range)
%   results = jury_routh_stability_sim(G, routh_range, jury_range, 'EpsFrac', 0.05)
%   results = jury_routh_stability_sim(G, routh_range, jury_range, 'Eps', 0.1)
%   results = jury_routh_stability_sim(..., 'SimTime', tFinal)
%
%   Given the stable-K range predicted by the Routh criterion and the
%   stable-K range predicted by the Jury criterion, this builds 4 test
%   gains per criterion:
%
%       below_lower  -> slightly UNDER the lower limit   (expected UNSTABLE)
%       above_lower  -> slightly ABOVE the lower limit    (expected STABLE)
%       below_upper  -> slightly UNDER the upper limit    (expected STABLE)
%       above_upper  -> slightly ABOVE the upper limit    (expected UNSTABLE)
%
%   For each test gain K, it closes the loop with unity negative feedback
%   using K as a proportional controller in series with the plant G:
%
%       T = feedback(K * G, 1)
%
%   and computes/plots the step response of T. G can be continuous
%   (Ts = 0) or discrete (Ts > 0); tf/zpk/ss objects are all accepted.
%
%   INPUTS
%     G           : plant transfer function (tf/zpk/ss, continuous or discrete)
%     routh_range : [K_low, K_high] stable range from the Routh criterion
%     jury_range  : [K_low, K_high] stable range from the Jury criterion
%
%   NAME-VALUE PAIRS
%     'EpsFrac' : fraction of each range's width used as the offset for
%                 the boundary test points (default 0.05)
%     'Eps'     : absolute offset to use instead of EpsFrac (overrides it)
%     'SimTime' : simulation duration (seconds for continuous G, or a
%                 time value for discrete G). If omitted, an automatic
%                 duration is used, with a safe fallback if a test point
%                 is unstable and step() cannot pick a sensible horizon.
%
%   OUTPUT
%     results : struct with fields "Routh" and "Jury". Each contains:
%         .range  : [K_low, K_high]
%         .eps    : offset actually used
%         .points : struct with fields below_lower/above_lower/
%                   below_upper/above_upper, each holding
%                   .K, .t, .y, .status ('inside'/'outside'), .isstable
%
%   Produces 3 figures:
%     1) All test points (both criteria) overlaid
%     2) Only the points classified INSIDE the stable range
%     3) Only the points classified OUTSIDE the stable range
%
%   Example:
%     G = tf(0.1, [1 -0.9], 0.1);        % discrete plant, Ts = 0.1 s
%     routh_range = [0, 5];
%     jury_range  = [0, 4.5];
%     results = jury_routh_stability_sim(G, routh_range, jury_range);
%
%   See also STEP, FEEDBACK, ISSTABLE

% ---- parse inputs ----
p = inputParser;
addParameter(p, 'EpsFrac', 0.05);
addParameter(p, 'Eps', []);
addParameter(p, 'SimTime', []);
parse(p, varargin{:});
epsFrac = p.Results.EpsFrac;
epsAbs  = p.Results.Eps;
simTime = p.Results.SimTime;

pointTypes = {'below_lower', 'above_lower', 'below_upper', 'above_upper'};

pointLabels = containers.Map(pointTypes, { ...
    'slightly UNDER lower limit', ...
    'slightly ABOVE lower limit', ...
    'slightly UNDER upper limit', ...
    'slightly ABOVE upper limit'});

pointMarkers = containers.Map(pointTypes, {'v', '^', 's', 'd'});
pointClassif = containers.Map(pointTypes, {'outside', 'inside', 'inside', 'outside'});

criteriaNames  = {'Routh', 'Jury'};
criteriaRanges = {routh_range, jury_range};
criteriaColors = containers.Map(criteriaNames, ...
    {[0 0.4470 0.7410], [0.8500 0.3250 0.0980]});   % blue, red

results = struct();

for c = 1:numel(criteriaNames)
    name = criteriaNames{c};
    rangeVec = criteriaRanges{c};
    lowK = rangeVec(1);
    highK = rangeVec(2);
    if highK <= lowK
        error('%s range must satisfy high > low (got [%.4g, %.4g]).', ...
            name, lowK, highK);
    end
    width = highK - lowK;
    if ~isempty(epsAbs)
        e = epsAbs;
    else
        e = epsFrac * width;
    end

    Kvals = struct( ...
        'below_lower', lowK - e, ...
        'above_lower', lowK + e, ...
        'below_upper', highK - e, ...
        'above_upper', highK + e);

    pointsData = struct();
    for k = 1:numel(pointTypes)
        pt = pointTypes{k};
        Kv = Kvals.(pt);
        T = feedback(Kv * G, 1);

        [t, y] = local_safe_step(T, simTime);
        stableFlag = local_check_stability(T, name, pt, Kv);

        pointsData.(pt).K = Kv;
        pointsData.(pt).t = t;
        pointsData.(pt).y = y;
        pointsData.(pt).status = pointClassif(pt);
        pointsData.(pt).isstable = stableFlag;

        if isnan(stableFlag)
            stableStr = 'N/A (internal delay - inspect plot)';
        else
            stableStr = num2str(stableFlag);
        end
        fprintf('%-6s | %-28s | K=%-10.4g | %-7s | isstable=%s\n', ...
            name, pointLabels(pt), Kv, pointClassif(pt), stableStr);
    end

    results.(name).range = rangeVec;
    results.(name).eps = e;
    results.(name).points = pointsData;
end

local_plot_responses(results, pointTypes, pointLabels, pointMarkers, ...
    criteriaColors, 'all', 'Closed-loop step responses: ALL test points');
local_plot_responses(results, pointTypes, pointLabels, pointMarkers, ...
    criteriaColors, 'inside', 'Closed-loop step responses: INSIDE stable range');
local_plot_responses(results, pointTypes, pointLabels, pointMarkers, ...
    criteriaColors, 'outside', 'Closed-loop step responses: OUTSIDE stable range');

end % ===== main function =====


% ------------------------------------------------------------------
function stableFlag = local_check_stability(T, name, pt, Kv)
% isstable() cannot evaluate systems with internal delays (common when G
% has delay/dead-time and gets wrapped in feedback()). In that case we
% can't automatically classify stability - we warn and return NaN so the
% caller knows to check the plotted step response instead, per MATLAB's
% own suggested workaround.
    try
        stableFlag = isstable(T);
    catch ME
        warning(['isstable failed for %s / %s (K=%.4g): %s\n' ...
            'Marking as unknown (NaN) - verify stability visually ' ...
            'from the step response plot.'], name, pt, Kv, ME.message);
        stableFlag = NaN;
    end
end


% ------------------------------------------------------------------
function [t, y] = local_safe_step(T, simTime)
% Computes a step response, falling back to a bounded horizon if the
% system is unstable and step() cannot auto-select a finite time.
    try
        if ~isempty(simTime)
            [y, t] = step(T, simTime);
        else
            [y, t] = step(T);
        end
    catch
        if T.Ts > 0
            tFinal = 30 * T.Ts;
        else
            tFinal = 5;
        end
        [y, t] = step(T, tFinal);
    end
end


% ------------------------------------------------------------------
function local_plot_responses(results, pointTypes, pointLabels, ...
    pointMarkers, criteriaColors, which, ttl)

    figure('Name', ttl, 'Color', 'w');
    hold on; grid on; box on;

    criteriaNames = fieldnames(results);
    legendEntries = {};

    for c = 1:numel(criteriaNames)
        name = criteriaNames{c};
        color = criteriaColors(name);
        pts = results.(name).points;

        for k = 1:numel(pointTypes)
            pt = pointTypes{k};
            status = pts.(pt).status;

            if strcmp(which, 'inside') && ~strcmp(status, 'inside')
                continue
            end
            if strcmp(which, 'outside') && ~strcmp(status, 'outside')
                continue
            end

            t = pts.(pt).t;
            y = pts.(pt).y;
            Kv = pts.(pt).K;

            if strcmp(status, 'inside')
                lineStyle = '-';
                lineWidth = 2.0;
            else
                lineStyle = '--';
                lineWidth = 1.5;
            end
            marker = pointMarkers(pt);
            nMk = max(numel(t), 1);
            mkIdx = unique(round(linspace(1, nMk, min(6, nMk))));

            plot(t, y, 'Color', color, 'LineStyle', lineStyle, ...
                'LineWidth', lineWidth, 'Marker', marker, ...
                'MarkerIndices', mkIdx, 'MarkerSize', 6, ...
                'MarkerFaceColor', color);

            legendEntries{end+1} = sprintf('%s: %s (K=%.3g, %s)', ...
                name, pointLabels(pt), Kv, status); %#ok<AGROW>
        end
    end

    xlabel('Time');
    ylabel('Output');
    title(ttl);
    if ~isempty(legendEntries)
        legend(legendEntries, 'Location', 'eastoutside', 'Interpreter', 'none');
    end
    hold off;
end