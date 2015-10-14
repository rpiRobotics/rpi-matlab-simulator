%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Give sphere S and polyhedron P, returns true if the sphere intersects the
% axis-aligned bounding box (AABB) of P.  

function intersect = cd_intersect_sphere_AABB( S, P )
    
    intersect = cd_sqDist_point_AABB(S.u,P) <= S.radius^2; 

end

