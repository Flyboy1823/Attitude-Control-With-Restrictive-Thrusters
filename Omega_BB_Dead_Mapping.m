function desired_thruster_config = Omega_BB_Dead_Mapping(torque_cmd, angular_velocity, w_des, omega_upper_bound)   
    desired_thruster_config = zeros(5,1);
    
    if torque_cmd(1) > 0 && (abs(w_des(1) - angular_velocity(1)) >= omega_upper_bound(1)) 
        desired_thruster_config(4)=torque_cmd(1); 
    end
    if torque_cmd(1) < 0 && (abs(w_des(1) - angular_velocity(1)) >= omega_upper_bound(1))
        desired_thruster_config(3)=abs(torque_cmd(1)); 
    end
    if torque_cmd(2) > 0 && (abs(w_des(2) - angular_velocity(2)) >= omega_upper_bound(2)) 
        desired_thruster_config(1)=torque_cmd(2); 
    end
    if torque_cmd(2) < 0 && (abs(w_des(2) - angular_velocity(2)) >= omega_upper_bound(2)) 
        desired_thruster_config(2)=abs(torque_cmd(2)); 
    end
end