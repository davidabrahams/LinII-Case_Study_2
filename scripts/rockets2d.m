function top_speed = rockets2d(mstage)

    time = 1500; % simulation time

    radius = 5; % the radius of the rocket
    Cd = 0.3; % the drag coefficient of the rocket
    payload = 1000; % the weight of the payload
    v_ex = 3000; %exhaust velocity in m/s
    flow = 122; % rate of fuel consumption kg/s
    p0 = 1.225;  % inital air denisty
    me = 5.972e24; % mass of earth
    G = 6.67384e-11;
    r_e = 6378e3; % radius of Earth
    a = pi * radius^2; % surface area of rocket
    mfuel_init = sum(mstage) * 4;
    atm_scale_height = 7000; % km
    stage_n = 1;

    Initial_Conidition = [r_e, 0, mfuel_init]; % Stage variables are height, velocity, and fuel


    function res = derivs(t, X)

        r = X(1);
        v = X(2);
        m_fuel = X(3);

        if m_fuel > 0
            Ft = v_ex * flow;
            dmdt = -flow;
        else
            m_fuel = 0;
            Ft = 0;
            dmdt = 0;
        end

        [stage_n, attached_stage_weight] = get_stage_n(mstage, m_fuel);
        % For some reason this still flickers.
%       disp([stage_n, attached_stage_weight, m_fuel]);
        m_total = payload + m_fuel + attached_stage_weight;

        h = r - r_e;
        p = p0 * exp(-h / atm_scale_height);

        drdt = v;
        Fg = -G * m_total * me / r^2;
        % TODO: CHECK IF THIS IS RIGHT!
        Fd = -(1/2) * Cd * p * v^2 * a;
        a = (Fg + Ft) / m_total;


        res = [drdt; a; dmdt];

    end

    function [value, terminal, direction] = events(~,W)
        r = W(1);
        v = W(2);

        h = r - r_e;
        value = h;
        direction = -1;
        terminal = 1;

    end


    options = odeset('Events', @events);
    [T, M] = ode45(@derivs, [0, time], Initial_Conidition, options);

    r = M(:,1) - r_e;
    v = M(:,2);
    fuel = M(:,3);

    clf;
    hold on;
    plot(T, v);

    top_speed = max(v);

end
