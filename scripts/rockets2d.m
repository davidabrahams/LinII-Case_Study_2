function top_speed = rockets2d(mstage)

    time = 1500; % simulation time

    radius = 5; % the radius of the rocket
    C_d = 0.3; % the drag coefficient of the rocket
    payload = 1000; % the weight of the payload
    v_exhaust = 3000; %exhaust velocity in m/s
    flow_rate = 122; % rate of fuel consumption kg/s
    p0 = 1.225;  % inital air denisty
    m_earth = 5.972e24; % mass of earth
    G = 6.67384e-11;
    r_earth = 6378e3; % radius of Earth
    rocket_area = pi * radius^2; % surface area of rocket
    m_fuel_init = sum(mstage) * 4;
    atm_scale_height = 7000; % km

    % Stage variables are height, velocity, and fuel
    Initial_Conidition = [r_earth, 0, m_fuel_init];

    % The derivs function to pass to ODE
    function res = derivs(~, X)
        
        % Unpack things
        height = X(1);
        velocity = X(2);
        m_fuel = X(3);
        
        % If there's fuel, use conservation of momentum to calculate force
        if m_fuel > 0
            Ft = v_exhaust * flow_rate;
            dmdt = -flow_rate;
            
        % otherwise, no thrust
        else
            m_fuel = 0;
            Ft = 0;
            dmdt = 0;
        end
        
        % The total mass of the ship is the mass of the fuel, payload, and
        % attached stages
        [~, attached_stage_weight] = get_stage_n(mstage, m_fuel);
        m_total = payload + m_fuel + attached_stage_weight;

        % calculate currnet air density based on height
        height_off_surface = height - r_earth;
        atm_density = p0 * exp(- height_off_surface / atm_scale_height);
        
        % calculate velocity and acceleration from forces
        F_g = -G * m_total * m_earth / height^2;
        % TODO: THIS IS BROKEN
        F_d = -(1/2) * C_d * atm_density * velocity^2 * rocket_area * ... 
            sign(velocity);
        rocket_area = (F_g + Ft) / m_total;

        res = [velocity; rocket_area; dmdt];

    end

    % This event happens when the rocket hits Earth on its way down
    function [value, terminal, direction] = events(~,W)
        
        height = W(1);

        h = height - r_earth;
        value = h;
        direction = -1;
        terminal = 1;

    end


    options = odeset('Events', @events);
    [T, M] = ode45(@derivs, [0, time], Initial_Conidition, options);

    % Unpack ODE
    Height = M(:,1) - r_earth;
    Velocity = M(:,2);
    Fuel = M(:,3);
    
    % Generate Attached Stage Weight
    attached_stage_weight = zeros(length(Fuel), 1);
    for i=1:length(attached_stage_weight)
        [~, attached_stage_weight(i, 1)] = get_stage_n(mstage, Fuel(i));
    end
    
    % Plot stuff
    clf;
    hold on;
    plot(T, Height);
    
    % Return the top speed
    top_speed = max(Velocity);

end
