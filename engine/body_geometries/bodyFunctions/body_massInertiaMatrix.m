

% Given a body struct B, return the world frame Mass-Inertia matrix
function M = body_massInertiaMatrix( B )

    % Update world frame inertia tensor
    R = qt2rot(B.quat);
    B.Jworld = R * B.J * R';
    M = [ B.mass*eye(3)   zeros(3)
             zeros(3)      B.Jworld ]; 

end

