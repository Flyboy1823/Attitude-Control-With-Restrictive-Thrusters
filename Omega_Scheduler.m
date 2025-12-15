function [thruster_state, total_impulse_accumulated_per_thruster, current_impulse_added_body]=Omega_Scheduler(thruster_state,desired_thruster_config, total_impulse_accumulated_per_thruster, F, dt, force_dir, t_min_on, t_max_on, t_min_off, thruster_on_time, thruster_off_time, current_impulse_added_body)
    for i=1:length(thruster_state)
        if thruster_state(i)==1
                            
            % Impulse Tracking
            dI = F(i) * dt;  % scalar impulse.  Subject to change with delayed dynamics of thrusters.  Can  be conditinoal on current thruster on time
            total_impulse_accumulated_per_thruster(i) = total_impulse_accumulated_per_thruster(i) + dI;
            current_impulse_added_body = current_impulse_added_body + dI * force_dir(i,:)';   % vector sum
            
            % Must keep firing check
            if thruster_on_time(i) < t_min_on
                continue; % do not allow turning off yet
            end

            % Shut off conditions
            if desired_thruster_config(i)<=0 || thruster_on_time(i) >= t_max_on
                thruster_state(i)=0; 
            end
        
        else
            % Turn On conditions
            if thruster_off_time(i)>=t_min_off && desired_thruster_config(i)>0
                thruster_state(i)=1; 
            end
        end
    end
    thruster_state(5) = 0;   % in this scheduler section, no need for center thruster


end