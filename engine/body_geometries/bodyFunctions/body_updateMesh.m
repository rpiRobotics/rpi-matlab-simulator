

% This function updates mesh-specific attributes of a body B

function B = body_updateMesh( B )

    % Update world coordinates
    for v = 1:B.num_verts
        B.verts_world(v,:) = B.u + qtrotate( B.quat, B.verts_local(v,:)' );
    end

    % Update face normals        TODO: normals and tvecs should be calculated only when needed
    for f = 1:B.num_faces
        v1 = B.verts_world( B.faces(f,1),: );
        v2 = B.verts_world( B.faces(f,2),: );
        v3 = B.verts_world( B.faces(f,3),: );
        n = cross3(v3-v2,v1-v2); 
        B.face_norms(f,:) = n / norm(n);
    end

    % Update t vectors 
    for e =1:B.num_edges
        E = B.verts_world(B.edges(e,2),:) - B.verts_world(B.edges(e,1),:);  
        if B.edges(e,3)
            B.tvecs(e,1:3) = cross3(B.face_norms(B.edges(e,3),:), E);  % t1
            B.tvecs(e,1:3) = B.tvecs(e,1:3) / norm(B.tvecs(e,1:3)); % This may not be strictly necessary, but let's normalize the t vecs
        end
        if B.edges(e,4)
            B.tvecs(e,4:6) = cross3(E, B.face_norms(B.edges(e,4),:));  % t2
            B.tvecs(e,4:6) = B.tvecs(e,4:6) / norm(B.tvecs(e,4:6)); 
        end
    end
    
    % Update bounding box 
    maxBBexpand = 0.05;      % TODO: replace hard-coded epsilons
    
    AABB_min = min(B.verts_world);
    AABB_max = max(B.verts_world); 
    vec = AABB_max-AABB_min;
    length = norm(vec);
    expansion = min(maxBBexpand, 0.15*length);   % Increase bounding box by 3% or maxBBexpand along diagonal.
    B.AABB_min = AABB_min - expansion*vec;     
    B.AABB_max = AABB_max + expansion*vec;    
    
end

