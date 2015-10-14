
% INPUT:
%   A   - A body
%   B   - A body
%   va  - Index of vertex on A
%   fb  - Index of face on B
%
% OUTPUT
%   APP - The applicability of va on fb

function APP = applicability_vf( A, B, va, fb )

    % Face Fb
    Fb = B.faces(fb,:);     
    nb = cross( B.verts_world(Fb(2),:) - B.verts_world(Fb(1),:), ...
                B.verts_world(Fb(3),:) - B.verts_world(Fb(2),:) );
    nb = nb/norm(nb); 

    % Vertex Va
    Va = A.verts_world(va,:);
    Vk = get_adjacent_vertices( A, va );  
    
    % Loop over adjacent vertices to find minimum dot product
    norm_edge = A.verts_world(Vk(1),:) - Va;
    norm_edge = norm_edge/norm(norm_edge); 
    APP = dot3(nb, norm_edge);
    for i=2:length(Vk)
        norm_edge = A.verts_world(Vk(i),:) - Va;
        norm_edge = norm_edge/norm(norm_edge); 
        d = dot3(nb, norm_edge);
        if d < APP 
            APP = d; 
        end
    end
    
end

