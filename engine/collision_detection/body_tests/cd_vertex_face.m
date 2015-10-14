%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Given two body structs, returns a set C of VERTEX-FACE contacts between them.

function C = cd_vertex_face( A, B, epsilon_vf, eps_theta )

    Aid = A.bodyID;
    Bid = B.bodyID; 

    % For now, brute-force
    C = [];

    %% vertex-face B1 on B2
    % For every vertex in B1
    for i = 1:A.num_verts
        v1 = A.verts_world(i,:);                       % The ith vertex of B1
        Vadj = get_adjacent_vertices( A, i );          % Vertex adjacency list

        % For every face in B2
        for j = 1:B.num_faces  
            f_j = B.faces(j,:);                        % The jth face of B2
            
            % This test is necessary for STEWART-TRINKLE
            if ~point_in_triangle_3d(v1, B.verts_world(f_j(1),:),...
                                         B.verts_world(f_j(2),:),...
                                         B.verts_world(f_j(3),:) )
                continue;
            end

            % VERTEX-FACE APPLICABILITY
            applicable = true;
            fnorm = B.face_norms(j,:); 
            for va =1:length(Vadj)
                edge = v1 - A.verts_world(Vadj(va),:);
                edge = edge / norm(edge); 
                if dot3( fnorm, edge ) > eps_theta
                    applicable = false; 
                    break; 
                end
            end
            if ~applicable, continue; end 

            % Another bounding check, to make sure v1 is within epsilon of the nearest point ON the triangle face
            Tri = B.verts_world([f_j(1) f_j(2) f_j(3)],:);
            if point_triangle_distance_3d(Tri, v1) < epsilon_vf
                v2 = B.verts_world(f_j(1),:);          % Arbitrary vert in f_j
                n = B.face_norms(j,:);
                psi = dot3(v1-v2, n);                   % Dot distance with face normal
                if psi > -.005   % TODO: remove hard-coded epsilon
                    C = [C Contact( Aid,Bid,v1,-n,psi )];
                end
            end

        end % for every face in B2
        
    end % for every vertex in B1

end






















