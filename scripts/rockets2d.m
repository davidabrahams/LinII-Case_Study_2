function res = rockets2d()
    radius = 0.2;
    Cd = 0.7;
    m_shell = 100;
    mfuel_init = 300; %kg
    v_ex = 10000; %m/s
    flow = 1; %kg/s
    g = 9.8;
    p0 = 1.225;
    me = 5.972e24;
    G = 6.67384e-11;
    r_e = 6371000;
    A = pi * radius^2;
    
    I = [r_e, 0, sind(89.4575), cosd(89.4575), mfuel_init]; %x,y,vx,vy,mfuel_init
   
    function dXdt = derivs(t, X)
        rx = X(1);
        ry = X(2);
        vx = X(3);
        vy = X(4);
        m = X(5);
       
        r_vec = [rx;ry]; % r = xi+yj
        r_hat = r_vec ./ norm(r_vec);
        v_vec = [vx; vy]; %v = vx;i + vyi 
        v_hat = v_vec ./ norm(v_vec);
        h = norm(r_vec) - r_e;
        p = p0*exp(-h/8000);
        
        if m_shell+m > m_shell
            Ft = (flow*v_ex)*v_hat;
            dmdt = -flow;
        else
            Ft = [0;0];
            dmdt = 0;
        end
        
        drxdt = vx;
        drydt = vy;
        Fg = (-G*(m+m_shell)*me/(norm(r_vec))^2)*r_hat;
        Fd = (-1/2*Cd*p*norm(v_vec)^2*A)*v_hat;
        a = (Fg + Fd + Ft)/(m+m_shell);
        %dvxdt = (-G*(m+m_shell)*me/(rx)^2*r_hat -1/2*Cd*p*vx^2*A + flow*v_ex)/(m+m_shell);
        %dvydt = (-G*(m+m_shell)*me/(ry)^2*r_hat -1/2*Cd*p*vy^2*A + flow*v_ex)/(m+m_shell);
     
        

        dXdt = [drxdt;drydt; a; dmdt];
        
    end
    
    function [value, terminal, direction] = events(time,W)
        rx = I(:,1);
        ry = I(:,2);
        
        r_vec = [rx; ry]; %v = vx;i + vyi 
        h = norm(r_vec) - r_e;
        p = p0*exp(-h/8000);
        value = h;
        direction = -1;
        terminal = 1;
        
    end

    time = 40000;
    options = odeset('Events', @events);
    [T, M] = ode45(@derivs,[0 time], I, options);
    
    rx_pos = M(:,1);
    ry_pos = M(:,2);
    vx_pos = M(:,3);
    vy_pos = M(:,4);
    m_fin = M(:,5);
    
    %v_pos(end);
    %m_fin(end)
    %vf = v_ex * log((mfuel_init+m_shell)/m_shell);
    
    hold on
    plot(rx_pos, ry_pos)
    dom = 0:2*pi/48:2*pi;
    xp = (r_e*cos(dom));
    yp = r_e*sin(dom);
    plot(xp,yp)
    axis equal
    %plot(vx_pos, vy_pos)
    legend('position')
    
    

end