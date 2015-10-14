%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Returns the squared distance from a point p to the AABB of a Body B.
%

function sqDist = cd_sqDist_point_AABB( p, B )

    sqDist = 0;
    
    % X distance
    if p(1) < B.AABB_min(1), sqDist = sqDist + (B.AABB_min(1)-p(1))^2; end
    if p(1) > B.AABB_max(1), sqDist = sqDist + (p(1)-B.AABB_max(1))^2; end
        
    % Y distance
    if p(2) < B.AABB_min(2), sqDist = sqDist + (B.AABB_min(2)-p(2))^2; end
    if p(2) > B.AABB_max(2), sqDist = sqDist + (p(2)-B.AABB_max(2))^2; end
        
    % Z distance
    if p(3) < B.AABB_min(3), sqDist = sqDist + (B.AABB_min(3)-p(3))^2; end
    if p(3) > B.AABB_max(3), sqDist = sqDist + (p(3)-B.AABB_max(3))^2; end
       
end

