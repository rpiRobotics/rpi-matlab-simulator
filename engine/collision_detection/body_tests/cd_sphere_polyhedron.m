%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Given bodies S and P (sphere and convex polyhedron), returns the single 
% nearest contact between the two if the gap distance is less than epsilon. 

function c = cd_sphere_polyhedron( S, P, epsilon )

    if S.bodyID == 2
       2;  
    end
    
    c = []; 
    
    % Compare sphere against every face (inefficient)
    max_distance = -inf; 
    max_distances = [];
    max_faces = [];
    for f = 1:P.num_faces
       distance = dot3( S.u' - P.verts_world(P.faces(f,1),:) , P.face_norms(f,:) );
       
       if distance > epsilon + S.radius, return; end
       
       if distance >= max_distance - 10^-5
          max_distance = distance; 
          max_faces = [max_faces f];
          max_distances = [max_distances distance]; 
       end
    end
    
    % Remove all but the largest sphere-face distances
    max_faces( max_distances < max_distance-10^-5 ) = [];
    
    % Faces may have angle between them of pi, resulting in the same
    % distance to the sphere.  So we must check all triangle faces against
    % the sphere to determine which it is actually nearest to (again inefficient).
    Ppoint = [0 0 0]; 
    min_psi = inf;
    for f = 1:length(max_faces)
        f_j = P.faces(max_faces(f),:); 
        Tri = P.verts_world([f_j(1) f_j(2) f_j(3)],:);
        [psi_f, pb] = point_triangle_distance_3d(Tri, S.u); 
        
        if psi_f < min_psi
           min_psi = psi_f; 
           Ppoint = pb; 
        end
    end
    
    normal = Ppoint - S.u';
    normal = normal / norm(normal); 
    pa = S.u' + S.radius * normal;
    
    c = Contact( S.bodyID, P.bodyID, pa, normal, min_psi-S.radius );

end

