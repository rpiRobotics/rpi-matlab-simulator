

function [C, Cdot] = joint_constraintError( sim, j )

    Jnt = sim.joints(j); 
    body1 = sim.bodies(Jnt.body1id);
    body2 = sim.bodies(Jnt.body2id);
    
    % Rotation error  THIS DOESN'T WORK...
    dtheta_x = 0; %sign(dot3( Jnt.Z2-Jnt.P2, Jnt.Y1-Jnt.Y2 )) * acos(dot3( Jnt.Y1-Jnt.P1, Jnt.Y2-Jnt.P2 ));
    dtheta_y = 0; %sign(dot3( Jnt.X2-Jnt.P2, Jnt.Z1-Jnt.Z2 )) * acos(dot3( Jnt.Z1-Jnt.P1, Jnt.Z2-Jnt.P2 ));
    dtheta_z = 0; %sign(dot3( Jnt.Y2-Jnt.P2, Jnt.X1-Jnt.X2 )) * acos(dot3( Jnt.X1-Jnt.P1, Jnt.X2-Jnt.P2 )); 
    
    % Constraint error
    C = [ Jnt.P1-Jnt.P2
          dtheta_x
          dtheta_y
          dtheta_z      ];
    
    C = C(Jnt.mask); 
    
    [G1c, G2c] = joint_Jacobians( sim, j );
    if ~body1.dynamic
      Cdot = G2c' * body2.nu;       % TODO!!! These nu should be at the joint origin
    elseif ~body2.dynamic
      Cdot = G1c' * body1.nu;
    else
      Cdot = [G1c;G2c]' * [body1.nu; body2.nu];
    end
    
    
    % This section was for joint constraint stabalization, and needs to go
    % there.
%     % Position error
%     C = [Jnt.P1-Jnt.P2; cross3(Jnt.Z1-Jnt.P1, Jnt.Z2-Jnt.P2)];  % TODO: I don't think the second half is correct
%     C = C(Jnt.mask); 
% 
%     % Velocity error
%     [G1c, G2c] = joint_Jacobians( sim, j );
%     if ~body1.dynamic
%       Cdot = G2c' * body2.nu;
%     elseif ~body2.dynamic
%       Cdot = G1c' * body1.nu;
%     else
%       Cdot = [G1c;G2c]' * [body1.nu; body2.nu];
%     end 

end

