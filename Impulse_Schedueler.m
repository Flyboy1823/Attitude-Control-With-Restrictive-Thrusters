function [new_thruster_state, current_group] = Impulse_Schedueler(thruster_state, thruster_on_time, thruster_off_time, t_min_on, t_max_on, t_min_off, current_group, groups)
    
    thruster_can_be_on=zeros(length(thruster_state), 1);
    thruster_has_to_be_on = zeros(length(thruster_state), 1);
    nGroups = length(groups); 

    % Check which thrusters have to be on and which ones can be on
    for  k=1:length(thruster_state)

        if (thruster_state(k)==1)  && (thruster_on_time(k) <= t_min_on)
            thruster_has_to_be_on(k) =1;
        end
        
        if ((thruster_state(k)==1) && (thruster_on_time(k) <=t_max_on)) || ((thruster_state(k)==0) && (thruster_off_time(k) >= t_min_off))
            thruster_can_be_on(k) = 1;
        end
    end

    new_thruster_state = thruster_has_to_be_on;

    % If there are thrusters that have to be on, check to see if their
    % opposites can turn on, and if so turn them on

    if sum(thruster_has_to_be_on) > 0
        if (thruster_has_to_be_on(1) && thruster_can_be_on(2))
            new_thruster_state(2) = 1;
        end
        if (thruster_has_to_be_on(2) && thruster_can_be_on(1))
            new_thruster_state(1) = 1;
        end
        if (thruster_has_to_be_on(3) && thruster_can_be_on(4))
            new_thruster_state(4) = 1;
        end
        if (thruster_has_to_be_on(4) && thruster_can_be_on(3))
            new_thruster_state(3) = 1;
        end
        
        % Current group doesn't need to change in this scenario
    
    % If no thruster has to fire
    else  
        % If there is a thruster that can fire
        if sum(thruster_can_be_on) > 0  
            
            found_group=false;
            
            % Starting from the current group, stop at the first group that
            % all of its thrusters can be on
            for offset = 0:(nGroups)-1
                g = mod(current_group-1 + offset, nGroups) + 1;
                thruster_g = groups{g};
                if all(thruster_can_be_on(thruster_g))
                    found_group=true;
                    break
                end
            end
            
            % new thruster state is the thrusters in the found group
            if found_group
                new_thruster_state(:) = 0;
                new_thruster_state(thruster_g) = 1;
                current_group = g;
            % if no group found, now thrusters are turned on
            else
                new_thruster_state(:)=0;
            end
        % if no thruster can be turned on
        else
            new_thruster_state(:)=0;
            % no need to change group since it will be changed next time
            % step
        end
    end
end






