

% Returns a contact struct C if the gap distance between sphere A and plane
% B is less than eps_sp.  
function C = cd_sphere_plane( A, B, eps_sp )

    C = []; 
    psi = dot3( A.u - B.u , B.plane_normal) - A.radius;
    if psi < eps_sp
       C = Contact( A.bodyID, B.bodyID, (A.u-A.radius*B.plane_normal)', -B.plane_normal', psi );
    end

end


