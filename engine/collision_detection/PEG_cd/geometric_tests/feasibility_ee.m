

function [FEAS, pa, pb, d] = feasibility_ee( A, B, ea, eb, eps_ee )

    FEAS = true; 
    
    edge_a = A.edges(ea,:);
    edge_b = B.edges(eb,:);

    va1 = A.verts_world(edge_a(1),:);
    va2 = A.verts_world(edge_a(2),:);
    vb1 = B.verts_world(edge_b(1),:);
    vb2 = B.verts_world(edge_b(2),:);
    
    [d,pa,pb] = segment_segment_distance_3d(va1,va2,vb1,vb2); 
    
    if d > eps_ee
       FEAS = false;
       return; 
    end
    
end

