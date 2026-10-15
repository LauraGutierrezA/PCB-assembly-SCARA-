function steps = aislarEscalonesPorEntrada(u, y, t, factorTiempoEstable)
    % AISLARESCALONESPORENTRADA Recorta datos de un experimento de respuesta al
    % escalón basándose UNICAMENTE en los cambios nítidos de la entrada 'u'.
    % Asegura un estado estable largo al final.
    
    arguments
        u (:,1) double
        y (:,1) double
        t (:,1) double
        factorTiempoEstable (1,1) double = 3.0 % Factor multiplicador para el tiempo estable.
    end

    % Asegurar vectores columna
    u = u(:); y = y(:); t = t(:);

    % --- 1. DETECCIÓN NITIDA DE LOS ESCALONES EN LA ENTRADA 'u' ---
    % Calculamos la diferencia absoluta entre puntos consecutivos de la entrada.
    du_abs = abs(diff(u));
    
    % Umbral de detección: cualquier cambio mayor al 10% del salto máximo.
    % Esto es muy robusto contra el ruido de medición.
    umbralCambioEntrada = 0.10 * max(du_abs);
    
    % Encontramos los índices donde la entrada cambia significativamente.
    indicesCambio = find(du_abs > umbralCambioEntrada);
    
    % --- 2. FILTRADO DE FALSOS POSITIVOS (ANTI-REBOTE) ---
    % A veces un solo escalón puede generar múltiples índices cercanos.
    % Nos quedamos solo con el primero de cada grupo de cambios.
    distanciaMinimaEntrePuntos = 10; % Número mínimo de puntos entre escalones.
    if ~isempty(indicesCambio)
        mask = [true; diff(indicesCambio) > distanciaMinimaEntrePuntos];
        indicesCambio = indicesCambio(mask);
    end
    
    numEscalonesDetectados = length(indicesCambio);

    % --- MENSAJE DE ESTADO EN CONSOLA ---
    fprintf('\n--- Detección de Escalones (Corregida) ---\n');
    fprintf('Se detectaron %d escalón(es) reales en la señal de entrada (u).\n', numEscalonesDetectados);

    if numEscalonesDetectados == 0
        error('No se detectó ningún cambio de escalón en el vector de entrada u. Verifica tus datos.');
    end

    steps = struct();

    % --- 3. PROCESAMIENTO Y RECORTE SEGURO DE CADA ESCALÓN ---
    % Iteramos a través de los escalones detectados.
    for k = 1:numEscalonesDetectados
        % Definimos el índice de inicio justo antes del cambio en 'u'.
        idxInicio = max(1, indicesCambio(k) - 1);
        t_inicio = t(idxInicio);

        % Lógica de recorte de tiempo fijo para estado estable:
        % Vamos a recortar un segmento que cubra la subida y luego un tiempo estable largo.
        
        % Definimos el límite de búsqueda para este segmento.
        if k < numEscalonesDetectados
            % Si hay otro escalón después, recortamos hasta justo antes de él.
            idxLimiteSegmento = indicesCambio(k+1) - 1;
        else
            % Si es el último escalón, recortamos hasta el final de los datos.
            idxLimiteSegmento = length(u);
        end
        
        % Extraemos el segmento de tiempo para este escalón.
        t_segmento_absoluto = t(idxInicio:idxLimiteSegmento);
        duracion_segmento = t_segmento_absoluto(end) - t_segmento_absoluto(1);
        
        % Estimamos una "constante de tiempo" (tau) muy conservadora.
        % Asumimos que la subida toma como máximo el 20% del tiempo total del segmento.
        tau_estimada = 0.20 * duracion_segmento;
        
        % Definimos el tiempo total de recorte que queremos:
        % 5 constantes de tiempo para la subida completa (asentamiento 99.3%)
        % + 5 constantes de tiempo multiplicado por el factor para un estado estable largo.
        tiempo_total_deseado = 5 * tau_estimada * (1 + factorTiempoEstable);
        
        % Buscamos el índice en el vector de tiempo absoluto que más se acerque a t_inicio + tiempo_total_deseado.
        t_fin_deseado = t_inicio + tiempo_total_deseado;
        
        % Aseguramos no pasarnos del límite de datos reales.
        if t_fin_deseado > t(idxLimiteSegmento)
             t_fin_deseado = t(idxLimiteSegmento);
        end
        
        % Encontramos el índice final exacto.
        [~, idxFinRelativo] = min(abs(t(idxInicio:idxLimiteSegmento) - t_fin_deseado));
        idxFin = idxInicio + idxFinRelativo - 1;

        % --- GUARDADO DE DATOS EN LA ESTRUCTURA ---
        nombrestep = sprintf('step%d', k);
        steps.(nombrestep).u = u(idxInicio:idxFin);
        steps.(nombrestep).y = y(idxInicio:idxFin);
        
        % Guardamos el tiempo relativo normalizado (empezando en t = 0)
        steps.(nombrestep).t = t(idxInicio:idxFin) - t_inicio;
        
        fprintf('  -> %s aislado: de t_abs=%.2f a t_abs=%.2f s (%d puntos).\n', ...
            nombrestep, t_inicio, t(idxFin), length(steps.(nombrestep).t));
    end
    fprintf('-------------------------------------------\n');
end