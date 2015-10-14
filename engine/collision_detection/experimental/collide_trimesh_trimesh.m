

function collide_trimesh_trimesh(B1, B2)

    B1id = B1.bodyID;
    B2id = B2.bodyID; 

    %% Constraint A : vertex-face B1 on B2
    % For every vertex in B1
    for i = 1:B1.num_verts
        v1 = B1.verts(i).world_coords;                    % The ith vertex of B1

        % For every face in B2
        for j = 1:B2.num_faces  
            f_j = B2.faces(j);                            % The jth face of B2
            v2 = B2.verts(f_j.verts(1)).world_coords;     % Arbitrary vert in f_j
            psi = dot3(v1-v2, f_j.normal);                % Gap distance

            if (norm(v1-B2.u) - B2.bound) < 0   % Check if vertex is within the bounding sphere of b2
               if psi > epsilon_minus && psi < epsilon_plus
                   % Another bounding check, to make sure v1 is within epsilon of the closest point ON the face
                   % Put triangle together 
                   Tri = [ B2.verts(f_j.verts(1)).world_coords ...
                           B2.verts(f_j.verts(2)).world_coords ...
                           B2.verts(f_j.verts(3)).world_coords ]' ;

                   triDist = pointTriangleDistance(Tri,v1');

                   if triDist < epsilon_plus   % triDist is always positive
                       sim.addContact(B1id,B2id,-f_j.normal, arbitraryTangent(f_j.normal), (v1-B1.u), (v1-psi*f_j.normal)-B2.u, psi);
                   end
               end
            end

        end % for every face in B2
    end % for every vertex in B1


    %% Constraint B : vertex-face B2 on B1
    % For every vertex in B2
    for i = 1:B2.num_verts
        v1 = B2.verts(i).world_coords;                    % The ith vertex of B2

        % For every face in B2
        for j = 1:B1.num_faces  
            f_j = B1.faces(j);                            % The jth face of B1
            v2 = B1.verts(f_j.verts(1)).world_coords;     % Arbitrary vert in f_j
            psi = dot3(v1-v2, f_j.normal);                % Gap distance

            if (norm(v1-B1.u) - B1.bound) < 0   % Check if vertex is within the bounding sphere of b2
               if psi > epsilon_minus && psi < epsilon_plus
                   % Another bounding check, to make sure v1 is within epsilon of the closest point ON the face
                   % Put triangle together 
                   Tri = [ B1.verts(f_j.verts(1)).world_coords ...
                           B1.verts(f_j.verts(2)).world_coords ...
                           B1.verts(f_j.verts(3)).world_coords ]' ;
                       
                   triDist = pointTriangleDistance(Tri,v1');
                   
                   if triDist < epsilon_plus   % triDist is always positive
                       sim.addContact(B1id,B2id,-f_j.normal, arbitraryTangent(f_j.normal), (v1-B2.u), (v1-psi*f_j.normal)-B1.u, psi);
                   end
               end
            end
            
        end % for every face in B2
    end % for every vertex in B1


    %% Constraint C : edge-edge
    % For every edge in B1
    for incr_eA = 1:B1.num_edges
      E1 = B1.edges(incr_eA);               % Edge
      e1 = B1.verts(E1.verts(2)).world_coords - B1.verts(E1.verts(1)).world_coords; % Edge vector
      tA1 = qtrotate(B1.quat, E1.t1);       % t vectors transformed to world coordinates
      tA2 = qtrotate(B1.quat, E1.t2);
      
      % For every edge in B2
      for incr_eB = 1:B2.num_edges

          % Calculate unsigned gap distance and nearest points on edges
          [d, ep1, ep2] = cda_calc_distance_edge_edge(B1, B2, incr_eA, incr_eB);

          if d > .1, continue; end  % TODO: a hard coded epsilon

          E2 = B2.edges(incr_eB); 
          e2 = B2.verts(E2.verts(2)).world_coords - B2.verts(E2.verts(1)).world_coords;
          tB1 = qtrotate(B2.quat, E2.t1);
          tB2 = qtrotate(B2.quat, E2.t2);

          % Calculate the normal of the edge-edge contact
          eX = cross3(e1,e2);       
          if norm(eX) == 0, continue; end       % Don't do parallel edges 
          n = sign( dot( eX, tB1+tB2) ) * eX ;
          n = n/norm(n);

          % Determine applicability APPL of edge-edge constraint
          ka1 = sign( dot(tA1, n) );    
          ka2 = sign( dot(tA2, n) );   
          kb1 = sign( dot(tB1, n) );   
          kb2 = sign( dot(tB2, n) );   
          if ka1==0 || ka2==0 || kb1==0 || kb2==0 
            continue; 
          end

          % The applicability does NOT determine penetration.  It only
          % decides if the edge-edge contact is feasible i.e. there is
          % the possibility of a dividing plane at the E-E contact.  
          APPL = (ka1==ka2 && kb1==kb2 && ka1~=kb1);

          a = B1.verts(E1.verts(1)).world_coords;
          b = B2.verts(E2.verts(1)).world_coords;  
          fq = dot( n, b-a );    %: f(q) determines penetration WHEN there is applicability

          psi = fq;

          sim.addContact(B1id,B2id,n,arbitraryTangent(n), ep1-B1.u, ep2-B2.u, psi);

      end
    end
end
