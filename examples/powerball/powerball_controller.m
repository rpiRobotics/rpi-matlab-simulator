%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = powerball_controller( sim )

    theta_desired = ones(length(sim.joints),1) * sin(sim.step*.01); 
    
    % Clear body torques
    for j=1:length(sim.joints)
       sim.bodies(sim.joints(j).body1id).Fext(4:6) = 0; 
       sim.bodies(sim.joints(j).body2id).Fext(4:6) = 0; 
    end
    
    % Calculate new torques
    for j=1:length(sim.joints)
       J = sim.joints(j);

       Perror = theta_desired(j) - J.theta;  % The proportional error
       Deriv  = (J.theta - J.theta_prev) / sim.h; 
       joint_frame_torque = 50*Perror - 5*Deriv; % PD controller

       t1 = J.T1world(1:3,1:3) * [0;0; -joint_frame_torque];  
       sim.bodies(J.body1id).Fext(4:6) = sim.bodies(J.body1id).Fext(4:6) + t1;

       t2 = J.T2world(1:3,1:3) * [0;0; joint_frame_torque];
       sim.bodies(J.body2id).Fext(4:6) = sim.bodies(J.body2id).Fext(4:6) + t2; 

       % For now, we need to keep track of this in the controller. 
       sim.joints(j).theta_prev = J.theta; 
    end

end

