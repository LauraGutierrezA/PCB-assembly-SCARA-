function TransientMetrics = analyzeProportionalController(G, K_inside, K_outside, step_amp)
    % ANALYZEPROPORTIONALCONTROLLER Simula la respuesta al escalón en lazo cerrado
    % 
    % Uso ejemplo:
    % Ts = 0.1; 
    % z = tf('z', Ts);
    % G = z^-6 *((0.005829*z+ 0.1605) / (z - 0.5916));
    % metrics = analyzeProportionalController(G, [0.5, 1.5, 3.0355], 3.1, 70);

    % Valores por defecto si no se ingresan todos los parámetros
    if nargin < 4
        step_amp = 70;
    end
    
    %% Define Proportional Gains (K)
    K_test = [K_inside, K_outside];

    %% Initialize Arrays for Transient Metrics
    Rise_Time = zeros(length(K_test), 1);
    Settling_Time = zeros(length(K_test), 1);
    Overshoot_Percent = zeros(length(K_test), 1);
    Peak_Value = zeros(length(K_test), 1);

    %% Simulate and Plot Step Responses
    figure('Name', 'Discrete Closed-Loop Step Responses');
    hold on;
    grid on;

    % Generate distinct colors for the plot lines
    colors = lines(length(K_test)); 

    for i = 1:length(K_test)
        K = K_test(i);

        % Closed-loop transfer function: T(z) = K*G(z) / (1 + K*G(z))
        T = feedback(K * G, 1);

        % 1. Simulate the standard unit step response
        [y_unit, t] = step(T, 15); 

        % 2. Scale the response for a step amplitude
        y = y_unit * step_amp;

        % 3. Extract and Store Transient Characteristics from the scaled data
        info = stepinfo(y, t);
        Rise_Time(i) = info.RiseTime;
        Settling_Time(i) = info.SettlingTime;
        Overshoot_Percent(i) = info.Overshoot;
        Peak_Value(i) = info.Peak;

        % 4. Plot results
        if K == K_outside
            plot(t, y, '--', 'Color', colors(i,:), 'LineWidth', 2, ...
                'DisplayName', sprintf('K = %g (Outside Range)', K));
        else
            plot(t, y, 'Color', colors(i,:), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('K = %g (Inside Range)', K));
        end
    end

    % Add a reference line to visualize the target
    plot(t, step_amp * ones(size(t)), 'k:', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Reference (%g)', step_amp));

    % Format the plot
    title('Discrete Closed-Loop Step Response');
    xlabel('Time (seconds)');
    ylabel('Amplitude');
    legend('Location', 'best');
    hold off;

    %% Display Transient Characteristics Table
    K_Value = K_test';
    TransientMetrics = table(K_Value, Rise_Time, Settling_Time, Overshoot_Percent, Peak_Value);
    disp('Transient Response Characteristics:')
    disp(TransientMetrics)
end