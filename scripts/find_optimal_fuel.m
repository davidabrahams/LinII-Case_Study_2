function res = find_optimal_fuel(num_stages)

    cost_per_kg = 5500;
    fuel_price = 2.72;
    cost_per_stage = 500e3;

    payload = 1000;
    
    
    
    x0 = ones(num_stages, 1);

    function res = optimize_cost(mstages)
        res = cost_per_kg*(5*sum(mstages) + payload) + fuel_price * ...
            sum(mstages)*4 + cost_per_stage*length(mstages);
    end

    lb = ones(num_stages, 1) * 20;
    ub = [];
    A = [];
    b = [];
    Aeq = [];
    Beq = [];
    
    res = fmincon(@optimize_cost, x0, A, b, Aeq, Beq, lb, ub, ...
        @get_top_speed);

end