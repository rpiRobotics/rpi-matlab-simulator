

% Returns true if the point p is in the interior of the triangle a,b,c when
% projected onto the plane of the triangle.  

function inside = point_in_triangle_3d( p, a,b,c )

    a = a-p;
    b = b-p;
    c = c-p;
    
    ab = dot3(a,b);
    ac = dot3(a,c);
    bc = dot3(b,c);
    cc = dot3(c,c);
    
    if bc*ac - cc*ab < 0
        inside = false;
    else
        bb = dot3(b,b);
        if ab*bc - ac*bb < 0
            inside = false;
        else 
            inside = true;
        end
    end

end

