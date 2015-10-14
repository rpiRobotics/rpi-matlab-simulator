
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic update with or without velocity
% Update the position and orientation of a body, including world coordinates and face normals.
% INPUTS: 
%       B       - A Body_mesh struct to be updated. 
%       x       - The body position and velocity 
%       newNu   - (optional) The body's velocity at the end of the current timestep.

function B = body_updateFromX( B, q, v)

    % Do not update static bodies
    %if ~B.dynamic, return; end

    % Velocity 
    B.nu = v;  

    % position
    B.u = q(1:3);   
    B.quat = q(4:7);
    B.quat = B.quat/norm(B.quat);
    
end























