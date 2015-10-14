
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computes qdot for a body 
% INPUTS: 
%       B       - A Body_mesh struct to be updated. 

function qdot = body_velocityToQdot( B, q, v )

    % Do not update static bodies
    %if ~B.dynamic, return; end

    % get angular velocity
    w = v(4:6);

    qdot(1:3) = v(1:3);
    qdot(4:7) = avel2qdot(q(4:7), w);

end























