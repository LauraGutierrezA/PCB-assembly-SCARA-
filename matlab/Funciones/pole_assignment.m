function [q, a_poles, Dz] = pole_assignment(G, tipo, zita, wn, Ts)
    % POLE_ASSIGNMENT Diseña controlador P, PI, PD o PID mediante asignación de polos.
    %
    % Entradas:
    %   G    : Función de transferencia de la planta en discreto (objeto tf)
    %   tipo : 'P', 'PI', 'PD' o 'PID'
    %   zita : Factor de amortiguamiento
    %   wn   : Frecuencia natural
    %   Ts   : Tiempo de muestreo
    %
    % Salidas:
    %   q       : Vector con los coeficientes del controlador [q0, q1, q2]
    %   a_poles : Vector con los polos rápidos (fijados y/o resueltos)
    %   Dz      : Función de transferencia en Z del controlador (objeto tf)
    %
    % Estructuras del controlador D(z) = Q(z)/P_den(z):
    %   P   : q0
    %   PI  : (q0*z + q1) / (z - 1)
    %   PD  : (q0*z + q1) / z            -> Kp = q0 + q1, Kd = -q1*Ts
    %   PID : (q0*z^2 + q1*z + q2) / (z*(z - 1))

    syms z real
    
    % 1. Extraer numerador (B) y denominador (A) de la planta G(z)
    [numG, denG] = tfdata(G, 'v');
    B = poly2sym(numG, z);
    A = poly2sym(denG, z);
    
    % Definir estructura del controlador D(z) = Q(z)/P_den(z)
    syms q0 q1 q2 real
    switch upper(tipo)
        case 'P'
            Q = q0;
            P_den = 1;
            q_vars = q0;
        case 'PI'
            Q = q0*z + q1;
            P_den = z - 1;
            q_vars = [q0, q1];
        case 'PD'
            Q = q0*z + q1;
            P_den = z;
            q_vars = [q0, q1];
        case 'PID'
            Q = q0*z^2 + q1*z + q2;
            P_den = z*(z - 1);
            q_vars = [q0, q1, q2];
        otherwise
            error('Tipo de controlador no válido. Usa P, PI, PD o PID.');
    end
    deg_Pden = length(q_vars) - 1;   % 0 para P, 1 para PI y PD, 2 para PID
    
    % Ecuación Característica (CE): P_den*A + Q*B = 0
    CE = P_den * A + Q * B;
    CE = collect(CE, z);
    
    % Extraer coeficientes de la CE para normalizar y obtener el grado
    ce_coeffs = coeffs(CE, z, 'All');
    n = length(ce_coeffs) - 1; % Grado de la ecuación característica
    
    % Normalizar dividiendo por el coeficiente de mayor grado
    CE_norm = collect(CE / ce_coeffs(1), z);
    C_CE = coeffs(CE_norm, z, 'All');
    
    % 2, 3, 4. Calcular los polos dominantes
    % Para zita > 1, sqrt(1 - zita^2) es imaginario y cos(jx) = cosh(x),
    % así que P_dom queda con los dos polos reales correctos.
    mag_z = exp(-zita * wn * Ts);
    Re = real(mag_z * cos(wn * Ts * sqrt(1 - zita^2)));
    
    % El par de polos dominantes: (z - p1)*(z - p2)
    P_dom = z^2 - 2*Re*z + (mag_z^2);
    
    % 5. Polos "rápidos" adicionales necesarios para igualar el grado de Pd al de CE
    num_f = n - 2; % Cantidad total de polos rápidos necesarios

    if num_f < 0
        error(['No es posible asignar 2 polos dominantes con el controlador "%s" ' ...
               'para esta planta (grado de la ecuación característica = %d < 2).'], tipo, n);
    end

    % --- Punto clave ---
    % El desbalance #incógnitas - #ecuaciones es exactamente (deg_Pden - 1).
    % Por lo tanto, hay que FIJAR exactamente max(deg_Pden - 1, 0) de los
    % polos rápidos, y dejar el resto como incógnitas para solve().
    % (PI y PD: deg_Pden = 1 -> no se fija ninguno)
    num_fixed = max(deg_Pden - 1, 0);
    num_free  = num_f - num_fixed;

    if num_free < 0
        error(['La planta es de orden demasiado bajo para el controlador "%s": ' ...
               'no hay suficientes polos rápidos disponibles para fijar los ' ...
               'grados de libertad redundantes.'], tipo);
    end

    % Polinomio de los polos rápidos que se resuelven
    F_free = z^num_free;
    f_vars = sym.empty(1,0);
    for i = 1:num_free
        f_sym = sym(sprintf('f%d', i), 'real');
        f_vars(end+1) = f_sym;
        F_free = F_free + f_sym * z^(num_free - i);
    end

    % Polinomio de los polos rápidos que se FIJAN automáticamente: cada uno
    % con parte real (equivalente continua) 10x más negativa que el anterior
    F_fixed = sym(1);
    for i = 1:num_fixed
        z_fast_i = mag_z^(10^i);
        F_fixed = F_fixed * (z - z_fast_i);
    end

    F_z = expand(F_free * F_fixed);
    
    Pd = collect(P_dom * F_z, z);
    C_Pd = coeffs(Pd, z, 'All');
    
    % Igualación de coeficientes (C_CE == C_Pd)
    max_len = max(length(C_CE), length(C_Pd));
    C_CE_pad = [zeros(1, max_len - length(C_CE)), C_CE];
    C_Pd_pad = [zeros(1, max_len - length(C_Pd)), C_Pd];
    
    eqs = (C_CE_pad == C_Pd_pad);
    
    % Resolver el sistema: #ecuaciones == #incógnitas (q_vars + f_vars libres)
    vars_to_solve = [q_vars, f_vars];
    sol = solve(eqs, vars_to_solve);
    
    if isempty(fieldnames(sol))
        error(['No se encontró solución. Verifica que el orden de la planta ' ...
               'sea compatible con el controlador "%s" elegido.'], tipo);
    end
    
    % Extraer los valores numéricos de q
    q = zeros(1, length(q_vars));
    for i = 1:length(q_vars)
        qi = sol.(char(q_vars(i)));
        if numel(qi) > 1
            qi = qi(1);   % sistema no lineal con más de una raíz: se toma la primera
        end
        q(i) = double(qi);
    end
    
    % Polos rápidos: los resueltos más los fijados directamente
    a_poles = [];
    if num_free > 0
        F_free_coeffs_val = [1, zeros(1, num_free)];
        for i = 1:num_free
            fi = sol.(char(f_vars(i)));
            if numel(fi) > 1, fi = fi(1); end
            F_free_coeffs_val(i+1) = double(fi);
        end
        a_poles = [a_poles, roots(F_free_coeffs_val).'];
    end
    for i = 1:num_fixed
        a_poles = [a_poles, double(mag_z^(10^i))];
    end
    
    % Construir la función de transferencia discreta del controlador (Dz)
    z_tf = tf('z', Ts);
    switch upper(tipo)
        case 'P'
            Dz = q(1);
        case 'PI'
            Dz = (q(1)*z_tf + q(2)) / (z_tf - 1);
        case 'PD'
            Dz = (q(1)*z_tf + q(2)) / z_tf;
        case 'PID'
            Dz = (q(1)*z_tf^2 + q(2)*z_tf + q(3)) / (z_tf * (z_tf - 1));
    end
end