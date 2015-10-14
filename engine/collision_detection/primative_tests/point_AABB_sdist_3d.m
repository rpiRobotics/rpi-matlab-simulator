

% Given a point p and an AABB, returns the squared distance 
% between p and the AABB represented by AABB_min and AABB_max.  

function sd = point_AABB_sdist_3d( p, AABB_min, AABB_max )
    
    sd = 0;
    for i=1:3
        v = p(i);
        if v < AABB_min(i) 
            sd = sd + (AABB_min(i) - v) * (AABB_min(i) - v);
        end
        if v > AABB_max(i)
            sd = sd + (v - AABB_max(i)) * (v - AABB_max(i));
        end
    end

end


