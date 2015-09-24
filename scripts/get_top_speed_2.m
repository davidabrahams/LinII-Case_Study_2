function [speed_diff, speed_eq] = get_top_speed_2(mstage)

    v_orbit = 7.8e3;
    payload = 1000; % the weight of the payload
    v_exhaust = 3000; %exhaust velocity in m/s
    
    delta_v = 0;
    
    for i=1:length(mstage)
        m_i = payload + 5 * sum(mstage(i:end));
        m_final = m_i - 4 * mstage(i);
        delta_v = delta_v + v_exhaust * log(m_i / m_final);
    end
    
    speed_diff = v_orbit - delta_v;
    speed_eq = [];

end

