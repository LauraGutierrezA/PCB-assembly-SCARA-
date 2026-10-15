function [q0, q1, q2, Dz] = discretePID(Kp, Taui, Taud, Ts, type)
    % discretePID Calcula los coeficientes q y la PTF Dz de un controlador discreto.
    %
    % Entradas:
    %   Kp, Taui, Taud : Parámetros del controlador (para P usar Taui = Inf, Taud = 0)
    %   Ts             : Tiempo de muestreo
    %   type           : 'P', 'PI', o 'PID'
    %
    % Salidas:
    %   q0, q1, q2     : Coeficientes de la ecuación en diferencias
    %   Dz             : Función de transferencia en Z (objeto tf)

    z = tf('z', Ts);

    switch upper(type)
        case 'P'
            q0 = Kp;
            q1 = 0;
            q2 = 0;
            Dz = q0;

        case 'PI'
            q0 = Kp * (1 + (Ts / (2 * Taui)));
            q1 = Kp * ((Ts / (2 * Taui)) - 1);
            q2 = 0;
            Dz = (q0 * z + q1) / (z - 1);

        case 'PID'
            q0 = Kp * (1 + (Ts / (2 * Taui)) + (Taud / Ts));
            q1 = Kp * ((Ts / (2 * Taui)) - 1 - (2 * Taud / Ts));
            q2 = Kp * (Taud / Ts);
            Dz = (q0 * z^2 + q1 * z + q2) / (z * (z - 1));

        otherwise
            error('Tipo de controlador no válido. Usa "P", "PI" o "PID".');
    end
end