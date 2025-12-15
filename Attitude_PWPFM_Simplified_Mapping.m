function desired_thruster_config = Attitude_PWPFM_Simplified_Mapping(torque_cmd)
    desired_thruster_config = zeros(5,1);
    if torque_cmd(1) > 0, desired_thruster_config(4)=torque_cmd(1); end
    if torque_cmd(1) < 0, desired_thruster_config(3)=abs(torque_cmd(1)); end
    if torque_cmd(2) > 0, desired_thruster_config(1)=torque_cmd(2); end
    if torque_cmd(2) < 0, desired_thruster_config(2)=abs(torque_cmd(2)); end
end