%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Given two body structs B1 and B2, returns true if their AABBs intersect.  
function intersect = cd_intersect_AABBs( B1, B2 )

    if any( B1.AABB_min > B2.AABB_max ) || any( B1.AABB_max < B2.AABB_min )
        intersect =  false;
    else
        intersect = true;
    end

end


