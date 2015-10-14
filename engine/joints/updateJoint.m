%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Inputs:
%           sim     - A Simulation object with at least jointIndex joints.
%           j       - An integer index of a joint
% Ouput:    
%           sim     - The simulation object with updated joint j

function sim = updateJoint( sim, j )

    body1 = sim.bodies(sim.joints(j).body1id);  
    body2 = sim.bodies(sim.joints(j).body2id); % TODO: if there is only one body, then this will break

    % Update body joint frames 
    T = zeros(4,4); 
    T(4,4) = 1;
    T(1:3,1:3) = qt2rot(body1.quat);
    T(1:3,4) = body1.u; 
    T = T*sim.joints(j).T1;
    sim.joints(j).P1 = T(1:3,4);
    sim.joints(j).X1 = T*[1;0;0;1]; sim.joints(j).X1 = sim.joints(j).X1(1:3);
    sim.joints(j).Y1 = T*[0;1;0;1]; sim.joints(j).Y1 = sim.joints(j).Y1(1:3);
    sim.joints(j).Z1 = T*[0;0;1;1]; sim.joints(j).Z1 = sim.joints(j).Z1(1:3);
    sim.joints(j).T1world = T;     % Stores T from world to joint for later use

    T = zeros(4,4); 
    T(4,4) = 1;
    T(1:3,1:3) = qt2rot(body2.quat);
    T(1:3,4) = body2.u; 
    T = T*sim.joints(j).T2;
    sim.joints(j).P2 = T(1:3,4);
    sim.joints(j).X2 = T*[1;0;0;1]; sim.joints(j).X2 = sim.joints(j).X2(1:3);
    sim.joints(j).Y2 = T*[0;1;0;1]; sim.joints(j).Y2 = sim.joints(j).Y2(1:3);
    sim.joints(j).Z2 = T*[0;0;1;1]; sim.joints(j).Z2 = sim.joints(j).Z2(1:3);
    sim.joints(j).T2world = T; 

    % Determine the current value of theta
    if sim.joints(j).jntCode == 2  % Revolute
       z1 = sim.joints(j).X1 - sim.joints(j).P1;
       z2 = sim.joints(j).X2 - sim.joints(j).P2; 
       sim.joints(j).theta = real( acos( dot(z1,z2) ) );
       if dot( sim.joints(j).Y1-sim.joints(j).P1 , sim.joints(j).X2-sim.joints(j).X1 ) < 0
          sim.joints(j).theta = -sim.joints(j).theta;  
       end
    elseif sim.joints(j).jntCode == 3 % Prismatic
       sim.joints(j).theta = norm(sim.joints(j).P2 - sim.joints(j).P1); 
    else
       %error('This joint type is currently not implemented.  I am sorry!');
    end

end

