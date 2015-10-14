
% Returns a vector t that is tangent to a normal vector n.
function t = arbitraryTangent(n) 
    if abs(dot3(n,[0;0;1])) < .7     % Tangent (kind of random)
        t = cross3(n,[0;0;1]);      % Is there a more elegant tangent here?
    else
        t = cross3(n,[0;1;0]);      
    end
    t = t/norm(t); 
end
    