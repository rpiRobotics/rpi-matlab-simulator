


function B = scale_mesh( B, scl )
    % Scale vertices
%     for i=1:B.num_verts
%         B.verts_local(i,:) = scl*B.verts_local(i,:);
%     end
    
    B.verts_local = B.verts_local * scl; 
    
    B = body_updatePosition(B,0);
    B = body_updateMesh(B); 

    % Scale mass ?
    B.mass = B.mass * scl;

    % Scale inertia matrix ? 
    B.J = B.J * scl;  

end

