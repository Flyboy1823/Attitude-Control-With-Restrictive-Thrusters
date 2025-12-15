%% Attitude Control Using Thruster Time Restrictions

clear; clc;
close all

%% Simulate Uncertainty?
sim=1;
MC = 10;
sim_ic = true;                   %change initial conditions each mc run?
close_to_target = false;
pertubation_sim = true;

if ~sim
    MC=1;
    sim_ic=0;
end
%% MC Start

for mc=1:MC
    fprintf("Running Monte-Carlo iteration %d/%d...\n", mc, MC);

    %% SPACECRAFT PARAMETERS
    I = diag([5, 5, 6]);   % Moment of inertia
    
    M = 10;                % Satellite Mass [kg]
    
    d = 0.5;               % Thruster arm distance (m)
    F = 0.5*ones(5,1);               % Thruster force magnitude (N)
    
    % Thruster locations comparable to CG
    thruster_pos = [ d,  0, 0;
                    -d,  0, 0;
                     0,  d, 0;
                     0, -d, 0;
                     0,  0, 0];
    
    thruster_dir = repmat([0 0 1],5,1); % all +z
    
    %% Uncertainties
    if sim
        sigma_I      = 0.05;    % 5% inertia uncertainty
        sigma_arm    = 0.02;    % 2% thruster arm position uncertainty
        sigma_F      = 0.10;    % 10% thrust magnitude uncertainty
        sigma_dir    = 0.02;    % 2% direction uncertainty (unit-vector renormalized)
        
        I = I .* (1 + sigma_I * randn(size(I)));
    
        thruster_pos = thruster_pos .* (1 + sigma_arm * randn(size(thruster_pos)));
    
        F = F .* (1 + sigma_F .* randn(size(F)));
    
         for i = 1:5
            perturb_dir = sigma_dir * randn(1,3);
            thruster_dir(i,:) = thruster_dir(i,:) + perturb_dir;
            thruster_dir(i,:) = thruster_dir(i,:) / norm(thruster_dir(i,:));  % renormalize
        end
    end
    
    
    %% Torque Each Thruster Produces
    
    force_dir = -thruster_dir;
    thruster_torque = zeros(5,3);
    for i = 1:5
        thruster_torque(i,:) = cross(thruster_pos(i,:), F(i)*force_dir(i,:));
    end
    
    % Max torque from single thruster
    max_torque = max(abs(thruster_torque),[],1);
    
    % Thruster Groupings of Torque Symmetry
    thruster_groups = { [1 2], [3 4], [1 2 3 4], [5]};
    current_group = 3;  % start with center thruster when moving for impulse
        
    % THRUSTER TIMING
    t_min_on  = 0.2;
    t_max_on  = 1.0;
    t_min_off = 5.0;
    
    %% INITIAL STATE
    % Attitude and Velocity Initial Conditions
    if sim_ic
        q_initial = randn(4,1);
        q_ib = q_initial;

        omega_mag = 0.5;   % rad/s (example)
        omega_dir = randn(3,1);
        omega_dir = omega_dir / norm(omega_dir);
        omega_0 = omega_mag * omega_dir;
        omega_0(3) = 0;
        angular_velocity = omega_0;
    else
        q_initial = [-1.3594; 0.9705; 0.1436; 0.0826];
        q_ib = q_initial;
        omega_0 =[0.2040; 0.3397; 0];  % Angular velocity (rad/s).  Z Angular Velocity is assumed 0 for this orientation
        angular_velocity = omega_0;
    end
    
    q_ib = q_ib/norm(q_ib);           % Normal Body to inertial quaternion (estimate provided by startrackers)
    q_bi = quatConj(q_ib);
    Pointing_Vector = [0; 0; -1];      % Front of Spacecraft direction (in body refrence frame)
    
    % Target Impulse
    Target_Impulse_Inertial_0 = [0; 0; -15];   % Total Impulse Vector to Add
    Target_Impulse_Inertial = Target_Impulse_Inertial_0;
    Target_Impulse_Body_q = quatMultiply(quatMultiply(q_bi, [0; Target_Impulse_Inertial]), quatConj(q_bi));
    Target_Impulse_Body_v = Target_Impulse_Body_q(2:end);
    impulse_accumulated_inertial_v = zeros(3,1);
    
    % Target Omega
    w_des = [0;0;0];
    
    % Error
    q_error = quatFromTwoVectors(Pointing_Vector, Target_Impulse_Body_v); % Calculate Error Quaternion (using updated impulse vectors)
    q_error = q_error/norm(q_error,2);
    
    % Thruster timers
    thruster_on_time = zeros(5,1);
    thruster_off_time = ones(5,1)*t_min_off;  
    thruster_state = zeros(5,1);    % 0=off, 1=on
    settling_time=[];
    total_thrust_target_reached_time=0; % Conditional on ending sim

    %% Sim Time and Frequency
    t = 0;
    dt = 0.02;  % 50 Hz
    tf = 400;

    
    %% Logging 
    log_t = [];
    log_w = [];
    log_thruster_states = [];
    log_quat_rot = [];
    log_q_ib = []; 
    log_impulse_per_thruster = [];
    log_Target_Impulse_Body_q= [];
    log_eul = [];
    log_x_i = [];
    log_y_i = [];
    log_impulse_accumulated_inertial_v = [];
    current_impulse_added_body = zeros(3,1);   % total impulse vector
    total_impulse_accumulated_per_thruster = zeros(5,1);   % per-thruster impulse magnitudes
    log_mode = 0;
    
    %% Control Paramaters
    Kp = .2;   % proportional gain on angle error
    Kd  = 20;   % derivative gain on angular velocity
    
    % Deadband for mitigating cascading thrusters
    deadband = t_min_on * max(max_torque(1:2));  % only X/Y axes
    I_diag=diag(I);
    omega_min= t_min_on * max_torque' ./ I_diag;
    theta_min = 0.5*(t_min_on^2) * max_torque' + [.2; .2; .2]*t_min_on;

    omega_upper_bound = 2.5*omega_min;
    omega_lower_bound = omega_min;
    theta_upper_bound = 5*theta_min;
    theta_lower_bound = 2.5*theta_min;

    theta_lower_bound_flag=0;
    
    %% Simulation Loop 
    while (t < tf  && total_thrust_target_reached_time < 6)

        % Current Error Quaternion Parameters
        q0 = q_error(1);
        q0 = min(1, max(-1, q0));
        theta = atan2(norm(q_error(2:end)), q0); % angle in radians between pointing and target
        axis_rot = q_error(2:end);                     % rotation axis
        axis_rot = axis_rot / norm(axis_rot);      % unit rotation axis        
        theta_dot = dot(axis_rot, angular_velocity);         % rate of change of angle

        % Inner Loop: If angular velocity not within tolarance
        if (abs(w_des(1) - angular_velocity(1)) > omega_upper_bound(1)) || (abs(w_des(2) - angular_velocity(2)) > omega_upper_bound(2))  
            
            log_mode(end+1) = 1;
            
            % PD Controller
            torque_cmd = Kp*sign(q_error(1))*q_error(2:end) + Kd*(w_des - angular_velocity);

            % Angular Velocity Bang Bang with Deadband Mapping 
            desired_thruster_config = Omega_BB_Dead_Mapping(torque_cmd, angular_velocity, w_des, omega_upper_bound);

            % Angular Velocity Scheduler
            [thruster_state, total_impulse_accumulated_per_thruster, current_impulse_added_body]=Omega_Scheduler(thruster_state,desired_thruster_config, total_impulse_accumulated_per_thruster, F, dt, force_dir, t_min_on, t_max_on, t_min_off, thruster_on_time, thruster_off_time, current_impulse_added_body);
        
            % Apply torque
            applied_torque = thruster_torque'*thruster_state;
                       
        % Middle loop: If attitude not within tolarance:                       
        else

            % Define Deadband and Hysteresis bounds
            if (abs(theta)<max(theta_upper_bound(1:2))) && (theta_lower_bound_flag==1)
                theta_bound=theta_upper_bound;
            else
                theta_bound=theta_lower_bound;
            end
            
            % If within bound and not out of fuel
            if (abs(theta) > max(theta_bound(1:2)))  && (sum(abs(impulse_accumulated_inertial_v)) < sum(abs(Target_Impulse_Inertial)))             
                
                log_mode(end+1) = 2;

                theta_lower_bound_flag=0;

                % SMC Controller
                torque_cmd= Attitude_SMC_Controller(angular_velocity, theta, q0, axis_rot, omega_lower_bound, deadband, I, omega_upper_bound, theta_bound);
                
                % Attitude PFPWM Mapper 
                desired_thruster_config = Attitude_PWPFM_Simplified_Mapping(torque_cmd);
            
                % Thruster Attitude Scheduler
                [thruster_state, total_impulse_accumulated_per_thruster, current_impulse_added_body]=Attitude_Scheduler(thruster_state,desired_thruster_config, total_impulse_accumulated_per_thruster, F, dt, force_dir, t_min_on, t_max_on, t_min_off, thruster_on_time, thruster_off_time, current_impulse_added_body);  
                
                % Apply torque
                applied_torque = thruster_torque'*thruster_state;

            % Outer Loop for Impulse Vector
            else
                if log_mode(end) == 2
                    current_group=3;
                end

                log_mode(end+1) = 3;
                theta_lower_bound_flag=1;

                if (sum(abs(impulse_accumulated_inertial_v)) < sum(abs(Target_Impulse_Inertial)))
                    
                    % Impulse Controller+Mapper+Scheduler
                    [thruster_state, current_group] = Impulse_Schedueler(thruster_state, thruster_on_time, thruster_off_time, t_min_on, t_max_on, t_min_off, current_group, thruster_groups);
             
                else % If we reached the the total impulse we want

                    log_mode(end) = 4;
                    total_thrust_target_reached_time=total_thrust_target_reached_time+dt;
                    
                    % Fire thrusters for minimum time
                    for k = 1:5
                        if thruster_state(k) == 1 && thruster_on_time(k) <= t_min_on
                            thruster_state(k) = 1;  % can't stop firing yet
                        else
                            thruster_state(k) = 0;
                        end
                    end

                    if sum(thruster_state)==0 && isempty(settling_time)
                        settling_time=t;
                    end
                                  
                end
                % Calculate applied impulse
                for i=1:5
                    if thruster_state(i)==1          
                        % Impulse Tracking
                        dI = F(i) * dt;  % scalar impulse (subject to change with dynamics of thrusters.  Can relate to current thruster on time)
                        total_impulse_accumulated_per_thruster(i) = total_impulse_accumulated_per_thruster(i) + dI;
                        current_impulse_added_body = current_impulse_added_body + dI * force_dir(i,:)';   % vector sum
                    end
                end
                % Calculate applied torque
                applied_torque = thruster_torque'*thruster_state;
            end
        end
    
        % Propagate Attitude
        if pertubation_sim
            sigma_tau    = 0.03;    % 3% pertubation torque in each axis
            sigma_impulse = 0.02;   % 2% impulse in each inertial direction
            applied_torque = applied_torque .* (1 + sigma_tau * randn(size(applied_torque)));
            current_impulse_added_body = current_impulse_added_body .* (1 + sigma_impulse * randn(size(current_impulse_added_body)));
        end

        w_dot = I\applied_torque; %I\(applied_torque - cross(angular_velocity, I*angular_velocity));
        angular_velocity = angular_velocity + w_dot*dt;
        
        omega = angular_velocity;  % angular velocity in body frame
        Omega = [0, -omega'; omega, -skew(omega)];
        q_dot = 0.5*Omega*q_ib;
        q_ib = q_ib + q_dot*dt;
        q_ib = q_ib / norm(q_ib);
        q_bi = quatConj(q_ib);
    
        % Propogate new target impulse vector 
        current_impulse_added_inertial = quatMultiply(quatMultiply(q_ib, [0; current_impulse_added_body]), quatConj(q_ib));
        current_impulse_added_inertial_v = current_impulse_added_inertial(2:end);
        impulse_accumulated_inertial_v = impulse_accumulated_inertial_v + current_impulse_added_inertial_v;
        if close_to_target
            Target_Impulse_Inertial = Target_Impulse_Inertial- impulse_added_inertial_v;  % This changes the inertial impulse vector each time stop given the current produced impulse
        end
        Target_Impulse_Body_q = quatMultiply(quatMultiply(q_bi, [0; Target_Impulse_Inertial]), quatConj(q_bi));
        Target_Impulse_Body_v = Target_Impulse_Body_q(2:end);
        
        q_error = quatFromTwoVectors(Pointing_Vector, Target_Impulse_Body_v); % Calculate Error Quaternion (using updated impulse vectors)
        q_error = q_error/norm(q_error,2);

        v_inertial_q = quatMultiply(quatMultiply(q_ib, [0; 0; 0; -1]),q_bi);
        v_inertial = theta*v_inertial_q(2:3)/norm(v_inertial_q(2:3));
        x_i = v_inertial(1);
        y_i = v_inertial(2);
        [eul1, eul2, eul3] = quat2angle(q_error');
        eul = [eul1, eul2, eul3];
               
        % Logging 
        log_t(end+1)      = t;
        log_w(:,end+1)    = angular_velocity;
        log_thruster_states(:,end+1)  = thruster_state;
        log_quat_rot(:,end+1) = q_error;
        log_impulse_per_thruster(:, end+1) = total_impulse_accumulated_per_thruster;
        log_impulse_accumulated_inertial_v(:, end+1) = impulse_accumulated_inertial_v;
        log_q_ib(:,end+1) = q_ib;
        log_eul(:,end+1) = rad2deg([eul1, eul2, eul3])';
        log_x_i(:, end+1) = x_i;
        log_y_i(:, end+1) = y_i;
        

        % Update thruster times
        for idx = 1:5
            if thruster_state(idx) == 1
                thruster_on_time(idx) = thruster_on_time(idx) + dt;
                thruster_off_time(idx) = 0;
            else
                thruster_off_time(idx) = thruster_off_time(idx) + dt;
                thruster_on_time(idx) = 0;
            end
        end

        % Reset 
        current_impulse_added_body = zeros(3,1);

        % Update Time
        t = t + dt;
    end

    MC_results(mc).omega                  = log_w;
    MC_results(mc).quat_rot           = log_quat_rot;
    MC_results(mc).quat_ib            = log_q_ib;
    MC_results(mc).final_impulse      = sum(total_impulse_accumulated_per_thruster);
    MC_results(mc).impulse_per_thr    = total_impulse_accumulated_per_thruster;
    MC_results(mc).thruster_impulse_history    = log_impulse_per_thruster;
    MC_results(mc).thruster_history   = log_thruster_states;
    MC_results(mc).t                  = log_t;
    MC_results(mc).q_initial          = q_initial;
    MC_results(mc).mode               = log_mode;
    MC_results(mc).eul                = log_eul;
    MC_results(mc).x_i                = log_x_i;
    MC_results(mc).y_i                = log_y_i;
    MC_results(mc).accumulated_inertial_impulse = log_impulse_accumulated_inertial_v; 
    MC_results(mc).final_total_sum_thruster_impulse =  sum(total_impulse_accumulated_per_thruster);
    MC_results(mc).total_final_offset_impulse_x = impulse_accumulated_inertial_v(1);
    MC_results(mc).total_final_offset_impulse_y = impulse_accumulated_inertial_v(2);
    MC_results(mc).total_final_offset_impulse_z = impulse_accumulated_inertial_v(3);
end
%% PLOTTING

% ---------------------- Angular Velocity -------------------------
figure;
axis_labels = {'x','y','z'};
for ax = 1:3
    subplot(3,1,ax); hold on;
    for mc = 1:MC
        t = MC_results(mc).t;
        angular_velocity = MC_results(mc).omega;
        plot(t, angular_velocity(ax,:));
    end
    ylabel(axis_labels{ax});
    if ax == 3, xlabel('Time (s)'); end
    axis tight
end
sgtitle('Angular Velocity (All MC Runs)');



% ---------------------- Eular Angles Offset -------------------------
figure;
axis_labels = {'x','y','z'};
for ax = 1:3
    subplot(3,1,ax); hold on;
    for mc = 1:MC
        t = MC_results(mc).t;
        angules = MC_results(mc).eul;
        plot(t, angules(ax,1:end));
    end
    ylabel(axis_labels{ax});
    if ax == 3, xlabel('Time (s)'); end
    axis tight
end
sgtitle('Eular Angles Offset [Deg]');

% ---------------------- Thruster States -------------------------
figure;
for thr = 1:5
    subplot(5,1,thr); hold on;
    for mc = 1:MC
        t = MC_results(mc).t;
        thr_hist = MC_results(mc).thruster_history;
        plot(t, thr_hist(thr,:));
    end
    ylabel(['T',num2str(thr)]);
    ylim([-0.2 1.2]);
    if thr==5, xlabel('Time (s)'); end
    axis tight
end
sgtitle('Thruster On/Off States (MC overlaid)');

% ---------------------- Error Quaternion -------------------------
figure;
q_labels = {'q_0','q_x','q_y','q_z'};
for qi = 1:4
    subplot(4,1,qi); hold on;
    for mc = 1:MC
        t = MC_results(mc).t;
        qr = MC_results(mc).quat_rot;
        plot(t, qr(qi,:));
    end
    ylabel(q_labels{qi});
    if qi == 4, xlabel('Time (s)'); end
    axis tight
end
sgtitle('Quaternion Error (All MC Runs)');

% ---------------------- Total Impulse -------------------------
figure; hold on;
for mc = 1:MC
    t = MC_results(mc).t;
    imp_hist = MC_results(mc).thruster_impulse_history;
    plot(t, sum(imp_hist,1));
end
xlabel('Time (s)');
ylabel('Total Impulse (N·s)');
title('Total Accumulated Impulse (MC overlaid)');
axis tight
grid on;

% ---------------------- Per-Thruster Impulse -------------------------
figure;
thruster_names = {'T1','T2','T3','T4','T5'};
for thr = 1:5
    subplot(5,1,thr); hold on;
    for mc = 1:MC
        t = MC_results(mc).t;
        imp_hist = MC_results(mc).thruster_impulse_history;
        plot(t, imp_hist(thr,:));
    end
    ylabel(thruster_names{thr});
    if thr==5, xlabel('Time (s)'); end
    axis tight
end
sgtitle('Impulse Accumulation by Thruster (MC)');


% ---------------------- Mode -------------------------
figure; hold on;
for mc = 1:MC
    t = MC_results(mc).t;
    mode = MC_results(mc).mode;
    plot(t, mode(2:end));
end
xlabel('Time (s)');
ylabel('1. Angular Velocity   2. Attitude   3. Impulse');
title('Mode');
axis tight
grid on;

% Angle of -Z vector in body frame relative to inertial
figure; hold on;

theta_circle = linspace(0,2*pi,400);

r1 = sin(theta_lower_bound(1));
r2 = sin(theta_upper_bound(1));

x1 = r1*cos(theta_circle);
y1 = r1*sin(theta_circle);
x2 = r2*cos(theta_circle);
y2 = r2*sin(theta_circle);

% ---- Draw shaded regions ONCE ----
patch(x2, y2, [0.8 0.8 1], ...
      'EdgeColor','none', 'FaceAlpha',0.3);

patch(x1, y1, [0.6 1 0.6], ...
      'EdgeColor','none', 'FaceAlpha',0.5);

% ---- MC scatter plots ----
for mc = 1:MC
    x_i = MC_results(mc).x_i;
    y_i = MC_results(mc).y_i;
    t   = MC_results(mc).t;

    scatter(x_i, y_i, 20, t, 'filled');
end

colorbar;
xlabel('Inertial X');
ylabel('Inertial Y');
axis equal;
grid on;
axis tight;
title('Body -Z Projection in Inertial XY (color = time)');


% Accumulated impulse in inertial refrence frame
figure;
ax = {'X','Y','Z'};
for thr = 1:3
    subplot(3,1,thr); hold on;
    for mc = 1:MC
        t = MC_results(mc).t;
        impulse = MC_results(mc).accumulated_inertial_impulse;
        plot(t, impulse(thr,:));
    end
    ylabel(ax{thr});
    if thr==3, xlabel('Time (s)'); end
    axis tight
end
sgtitle('Accumulated Impulse In Inertial Refrence Frame [Ns]');


%% FUNCTIONS
function S = skew(v)
    S = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2) v(1)   0 ];
end


function q = quatFromTwoVectors(v, w)
% q (4x1) rotates v (3x1) into w (3x1). Quaternions are [w; x; y; z].
% robust to parallel/anti-parallel inputs. v,w need not be normalized.

    % normalize
    v = v / norm(v);
    w = w / norm(w);

    % dot and cross
    d = dot(v,w);
    c = cross(v,w);
    c_norm = norm(c);

    % Numerical thresholds
    eps_ang = 1e-10;

    if c_norm > eps_ang
        % regular case
        axis = c / c_norm;
        theta = atan2(c_norm, d); % better numeric stability
        q = [cos(theta/2);
             axis * sin(theta/2)];
    else
        % cross is ~0 -> vectors are parallel or anti-parallel
        if d > 0.999999  % nearly identical
            q = [1; 0; 0; 0]; % identity
        else
            % opposite vectors: need 180 deg rotation about any axis
            % orthogonal axis: pick one component with smallest abs and swap
            % e.g. pick axis = unit( [1,0,0] x v ) or similar
            tmp = abs(v);
            [~, idx] = min(tmp);
            e = zeros(3,1); e(idx) = 1;
            axis = cross(v, e);
            axis = axis / norm(axis);
            q = [0; axis]; % cos(90)=0, sin(90)=1 => 180deg quaternion
        end
    end
    % normalize quaternion to be safe
    q = q / norm(q);
end

function q = quatConj(qin)
    q = [qin(1); -qin(2:4)];
end

function qp = quatMultiply(q1, q2)
% Hamilton product q1 * q2, quaternions as column [w; x; y; z]
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);
    w = w1*w2 - dot(v1,v2);
    v = w1*v2 + w2*v1 + cross(v1,v2);
    qp = [w; v];
end


