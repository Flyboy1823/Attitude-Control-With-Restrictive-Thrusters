function torque_cmd= Attitude_SMC_Controller(angular_velocity, theta, q0, axis_rot, omega_lower_bound, deadband, I, omega_upper_bound, theta_bound)
    torque_cmd = zeros(3,1);
    for i=1:2
        % If moving in the right direction
        if sign(angular_velocity(i)) == sign(q0*axis_rot(i))                
            % If far away
            if abs(theta*axis_rot(i)) > max(theta_bound)     
                % If moving too slow 
                if abs(angular_velocity(i)) < omega_lower_bound(i)
                    % Add speed in the same direction of motion
                    torque_cmd(i) = sign(angular_velocity(i))*deadband/I(i,i); 
                else
                    % If moving fast enough, don't do anything
                    torque_cmd(i) = 0;              
                end
    
            % If close and moving in right direction
            else   
                if abs(angular_velocity(i)) < omega_upper_bound(i)
                    torque_cmd(i) = 0;
                else
                    torque_cmd(i) = sign(angular_velocity(i))*deadband/I(i,i);
                end
            end
        % If not moving in the right direction
        else   
            % If far away
            if abs(theta*axis_rot(i)) > max(theta_bound)     
                torque_cmd(i) = -sign(angular_velocity(i))*deadband/I(i,i);
            else
                if abs(angular_velocity(i)) < omega_lower_bound(i)
                    torque_cmd(i) = 0;
                else
                    torque_cmd(i) = -sign(angular_velocity(i))*deadband/I(i,i);
                end
            end
        end   
    end



end