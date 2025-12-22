function u = ctrl_reactive(u_nom, laser_ranges, params)
%CTRL_REACTIVE  Capa reactiva robusta (ranges) con:
% - limpieza + saturación
% - sectores front/left/right
% - ESCAPE con memoria de lado
% - frenado suave por "corridor" frontal
% - evasión suave robusta (percentil) solo cuando hace falta

    persistent escape_count escape_dir pref_dir pref_timer
    if isempty(escape_count), escape_count = 0; end
    if isempty(escape_dir),   escape_dir   = +1; end  % +1 izq, -1 der
    if isempty(pref_dir),     pref_dir     = 0;  end  % 0 none, +1 izq, -1 der
    if isempty(pref_timer),   pref_timer   = 0;  end  % pasos restantes

    v_nom = u_nom(1);
    w_nom = u_nom(2);

    r = laser_ranges(:);
    N = numel(r);
    if N < 5
        u = u_nom;
        return;
    end

    % --- limpiar medidas ---
    r(~isfinite(r)) = params.range_max;
    r = min(max(r, params.range_min), params.range_max);

    % filtro simple anti-spikes (suave, no "mata" bordes)
    r = movmedian(r, 5);

    ang = linspace(params.ang_min, params.ang_max, N)';

    % --- Sectores ---
    front_narrow = abs(ang) <= params.front_half_angle;                 % frente angosto
    front_wide   = abs(ang) <= min(params.side_angle, 2*params.front_half_angle); % opcional
    left  = ang >  params.front_half_angle & ang <= params.side_angle;
    right = ang < -params.front_half_angle & ang >= -params.side_angle;

    % Distancias robustas por sector (percentil 10%)
    d_front = safeQuantile(r, front_narrow, 0.10, params.range_max);
    d_left  = safeQuantile(r, left,         0.10, params.range_max);
    d_right = safeQuantile(r, right,        0.10, params.range_max);

    % ============================================================
    % 0) ESCAPE: mantener giro por varios pasos
    % ============================================================
    if escape_count > 0
        escape_count = escape_count - 1;

        % retrocede suave si sigue muy cerca
        if d_front < params.d_min
            v_cmd = params.v_back;
        else
            v_cmd = 0.0; % despega y reorienta sin avanzar
        end

        w_cmd = escape_dir * params.w_escape;

        u = [clamp(v_cmd, params.v_min, params.v_max);
             clamp(w_cmd, -params.w_max, params.w_max)];
        return;
    end

    % ============================================================
    % 1) Emergencia: activar escape si frente demasiado cerca
    % ============================================================
    if d_front <= params.d_min
        % elegir el lado más libre (robusto)
        if d_left > d_right
            escape_dir = +1;
        else
            escape_dir = -1;
        end

        escape_count = params.escape_steps;

        % primera acción inmediata
        if d_front <= params.d_stop
            v_cmd = params.v_back;
        else
            v_cmd = 0.0;
        end
        w_cmd = escape_dir * params.w_escape;

        u = [clamp(v_cmd, params.v_min, params.v_max);
             clamp(w_cmd, -params.w_max, params.w_max)];
        return;
    end

    % ============================================================
    % 2) Frenado suave: reduce v si te acercas al obstáculo frontal
    % ============================================================
    if d_front < params.d_slow
        alpha = (d_front - params.d_min) / max(params.d_slow - params.d_min, 1e-6);
        alpha = clamp(alpha, 0, 1);

        % No dejes que el robot se "muera" demasiado si el nominal es alto
        v_cmd = alpha * min(max(params.v_slow, 0.05), max(v_nom, 0.0));
    else
        v_cmd = v_nom;
    end

    % ============================================================
    % 3) Evasión suave: SOLO si estás relativamente cerca
    %    + histeresis de lado para evitar zig-zag
    % ============================================================
    near = (d_front < params.d_slow) || (min(d_left, d_right) < params.d_slow);

    if pref_timer > 0
        pref_timer = pref_timer - 1;
    else
        pref_dir = 0;
    end

    w_avoid = 0.0;
    if near
        % error lateral robusto: si right está más cerca => gira a la izquierda, etc.
        side_err = (1/max(d_right,1e-3)) - (1/max(d_left,1e-3));

        % si aún no hay preferencia, decide un lado y mantenlo un rato
        if pref_dir == 0
            if d_left > d_right
                pref_dir = +1;
            else
                pref_dir = -1;
            end
            pref_timer = 8; % ~0.8s si dt=0.1
        end

        % fuerza el giro al lado preferido (reduce oscillation)
        w_avoid = params.k_w * side_err;

        % "empujón" mínimo hacia el lado preferido cuando muy parejo
        w_avoid = w_avoid + pref_dir * 0.10;
    end

    w_cmd = w_nom + w_avoid;

    % saturar
    u = [clamp(v_cmd, params.v_min, params.v_max);
         clamp(w_cmd, -params.w_max, params.w_max)];
end

% ---- helpers ----
function d = safeQuantile(r, mask, q, fallback)
    if any(mask)
        x = r(mask);
        x = sort(x);
        idx = max(1, min(numel(x), round(q * numel(x))));
        d = x(idx);
    else
        d = fallback;
    end
end

function y = clamp(x,a,b)
    y = min(max(x,a),b);
end
