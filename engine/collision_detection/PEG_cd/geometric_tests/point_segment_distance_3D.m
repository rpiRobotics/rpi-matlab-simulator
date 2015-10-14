

% Returns the distance from point c to segment ab.  Also returns the
% nearest point P on ab to c.  
function [ D, P ] = point_segment_distance_3D( a, b, c )

    e = b-a;                    % Edge vector e
    n = e/norm(e);              % Normalized e
    P = a + dot(c-a,n)*n;       % Point projected on line ab
    
    t0 = dot(e,P-a)/dot(e,e);   % Parameterized position on ab
    if t0 <= 0
        P = a;
        D = norm(c-a);
    elseif t0 >= 1
        P = b;
        D = norm(c-b);
    else
        D = norm(c-P);
    end

end

