function res = find_optimal_fuel(num_stages)

    payload = 1000;
    
    x0 = ones(num_stages, 1);

    function res = optimize_cost(mstages)
        res = 5500*(5*sum(mstages) + payload) + 2.72 * sum(mstages)*4 + 500000*length(mstages);
    end
    
    
    x = fmincon(@optimize_cost, x0, a, b, Aeq, Beq, lb, ub, @rockets2d);




end