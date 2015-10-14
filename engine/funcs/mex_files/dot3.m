
% Simple dot product of two vectors u and v, both assumed to be column
% vectors of length three.  This is meant to avoid calling MATLAB's built
% in dot function which does size checks.  
function r = dot3( u, v )
    r = u(1)*v(1) + u(2)*v(2) + u(3)*v(3); 
end

