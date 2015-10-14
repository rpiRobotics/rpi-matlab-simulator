%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

% Given mesh M and plane P, returns a set of contacts for all vertices of M
% within distance eps_vf of P. 
function C = cd_mesh_plane( M, P, eps_vf )

    C = []; 
    PSI = [];
    
    for v = 1:M.num_verts
        V = M.verts_world(v,:); 
        psi = dot3( V' - P.u , P.plane_normal);
        if psi < eps_vf
           C = [C Contact( M.bodyID, P.bodyID, V, -P.plane_normal', psi )];
           PSI = [PSI; psi]; 
        end
    end

    % Let's not return more than 4 contact points for mesh-plane
    % TODO: This is a nice idea, but is really not robust.  The points
    % chosen should be "nicely" distributed over the mesh so as to avoid
    % instability.  Consider the case where a mesh sits on a plane with
    % many vertices near the plane.  It is likely that a "bounce" will be
    % induced such that the object oscillates with one side nearest the
    % plane then the opposite side (after correction).  
    if length(C) > 4
       psi_sorted = sort(PSI);
       C( PSI > psi_sorted(4) ) = [];
    end
    
end

