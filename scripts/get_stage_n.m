function [stage_n, attached_stage_weight] = get_stage_n(stage_masses, ... 
    current_fuel_mass)

    temp_mass = current_fuel_mass;
    stage_n = length(stage_masses) + 1;
    for i = length(stage_masses):-1:1
        if temp_mass > 0
            stage_n = stage_n -1 ;
            temp_mass = temp_mass - 4 * stage_masses(i);
        else
            break;
        end
    end
    
    if stage_n > length(stage_masses)
        attached_stage_weight = 0;
    else
        attached_stage_weight = sum(stage_masses(stage_n:end));

end