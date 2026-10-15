function [Kp, Ti, Td, aplica] = ZN(type, tipo_ctrl, FT_ol, Ts)
    % type: 'Ultimate Gain' o 'Reaction Curve'
    % tipo_ctrl: 'P', 'PI', 'PID'
    % FT_ol: Función de transferencia en lazo abierto (objeto tf)
    % Ts: Tiempo de muestreo
    %
    % aplica (opcional, 4to output): true/false, indica si el método fue
    % aplicable. Si es false, Kp=0, Ti=Inf, Td=0 (controlador nulo) para
    % que el flujo de cálculo pueda continuar sin detenerse.

    aplica = true; % se vuelve false solo si alguna verificación de aplicabilidad falla

    switch type
        case 'Ultimate Gain'
            [Gm, ~, Wcg, ~] = margin(FT_ol);

            if isinf(Gm) || isempty(Gm) || isnan(Gm)
                warning('ZN:NoUltimateGain', ...
                    'La fase de la FT no cruza los -180 grados. Ultimate Gain no aplica. Se retorna controlador nulo.');
                [Kp, Ti, Td] = deal(0, Inf, 0);
                aplica = false;
                return;
            end

            Wu = Wcg;
            Gmu = 20 * log10(Gm);
            Ku = 10^(Gmu/20);
            Pu = 2*pi / Wu;

            switch upper(tipo_ctrl)
                case 'P'
                    Kp = 0.5 * Ku;  Ti = Inf;  Td = 0;
                case 'PI'
                    Kp = 0.45 * Ku; Ti = (1/1.2) * Pu; Td = 0;
                case 'PID'
                    Kp = 0.6 * Ku;  Ti = 0.5 * Pu; Td = 0.125 * Pu;
                otherwise
                    error('Controlador no válido. Usa P, PI o PID.');
            end

        case 'Reaction Curve'
            % Requiere FT_ol CONTINUO (K/(tau*s+1), InputDelay en segundos)
            if FT_ol.Ts ~= 0
                error('ZN:ReactionCurveRequiresContinuous', ...
                    ['La curva de reacción requiere el modelo FOPDT en tiempo CONTINUO ' ...
                     '(tau y theta en segundos). Recibiste un sistema discreto con Ts=%.4g. ' ...
                     'Pasa tu FOPDT continuo (antes de c2d/absorbDelay), no la planta discretizada.'], FT_ol.Ts);
            end

            K = dcgain(FT_ol);
            den = FT_ol.den{1};
            tau = den(1) / den(2);
            theta = FT_ol.InputDelay;
            thetap = theta + Ts/2;

            relacion = thetap / tau;
            if relacion <= 0.1 || relacion >= 1
                warning('ZN:ReactionCurveNotApplicable', ...
                    'No aplica el método de curva de reacción: thetap/tau (%.4f) no está entre 0.1 y 1. Se retorna controlador nulo.', relacion);
                [Kp, Ti, Td] = deal(0, Inf, 0);
                aplica = false;
                return;
            end

            switch upper(tipo_ctrl)
                case 'P'
                    Kp = tau / (K * thetap); Ti = Inf; Td = 0;
                case 'PI'
                    Kp = (0.9 * tau) / (K * thetap); Ti = 3 * thetap; Td = 0;
                case 'PID'
                    Kp = (1.2 * tau) / (K * thetap); Ti = 2 * thetap; Td = thetap / 2;
                otherwise
                    error('Controlador no válido. Usa P, PI o PID.');
            end

        otherwise
            error('Tipo no válido. Usa "Ultimate Gain" o "Reaction Curve".');
    end
end