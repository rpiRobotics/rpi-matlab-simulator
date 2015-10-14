%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% collide_poly_poly.m
%

function collide_poly_poly( sim, b1ID,b2ID, B1,B2 )

% Bools for which collision detection techniques to use
DO_VERTEX_FACE = 1;
DO_EDGE_EDGE = 1;
%DO_FACE_FACE = 0; 

EDGE_DEBUG = 0; 

epsilon_plus = .5;     % maximum distance (upper threshold for creating a contact)
epsilon_minus = -.3;   % minimum NEGATIVE distance (lower threshold for creating a contact)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% VERTEX-FACE
% FIXME: These epsilon could be dynamic wrt body-size and/or velocity

if DO_VERTEX_FACE

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
                       sim.addContact(b1ID,b2ID,-f_j.normal, arbitraryTangent(f_j.normal), (v1-B1.u), (v1-psi*f_j.normal)-B2.u, psi);
                   end
               end
            end
            
        end % for every face in B2
    end % for every vertex in B1
    
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
                       sim.addContact(b2ID,b1ID,-f_j.normal, arbitraryTangent(f_j.normal), (v1-B2.u), (v1-psi*f_j.normal)-B1.u, psi);
                   end
               end
            end
            
        end % for every face in B2
    end % for every vertex in B1


end % End VERTEX_FACE


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% EDGE-EDGE collision
% refer to [Ebe08b], cited on p116 of Binh's disseration

if (DO_EDGE_EDGE)
    
B1id = B1.bodyID;   % TODO: bodyID is too confusing with BodyIndex
B2id = B2.bodyID; 
    
if B1id < B2id  % Avoid redundant edge-edge contacts.  TODO: I think this should 
    % actually be handled by separating v-f and e-e into two files.  
    
    % for every edge in B1, B2
    for incr_eA = 1:B1.num_edges
      for incr_eB = 1:B2.num_edges
          
        % Completely skip edges that simply divide a face i.e. the angle
        % between the faces is PI.  
        % TODO: This may very well break some things...
        if B1.edges(incr_eA).edge_angle == 0 || B2.edges(incr_eB).edge_angle == 0
            continue;
        end
          
        % calculate gap distance and closest points on the edges
        [psi, ep1, ep2] = cda_calc_distance_edge_edge(B1, B2, incr_eA, incr_eB);
        
        % Check if ep1 or ep2 is a vertex, if it is then disregard it
        if     ep1 == B1.verts(B1.edges(incr_eA).verts(1)).world_coords, continue;   
        elseif ep1 == B1.verts(B1.edges(incr_eA).verts(2)).world_coords, continue;
        elseif ep2 == B2.verts(B2.edges(incr_eB).verts(1)).world_coords, continue; 
        elseif ep2 == B2.verts(B2.edges(incr_eB).verts(2)).world_coords, continue;
        end
   
        % If contact is within bounds (NOTE: psi is always positive here)
        % TODO: If we really want different eps- and eps+, we should check
        % again below after the does_edge_intersect_face checks.  
        %if psi > epsilon_minus && psi < epsilon_plus    
        if psi < 0.05    % TODO: Use a small but reasonable eps for this particular check
            
            % Determine the correct sign of psi
            %p12 = ep2 - ep1;  % Un-normalized normal vector
            va1 = B1.verts(B1.edges(incr_eA).verts(1)).world_coords; 
            va2 = B1.verts(B1.edges(incr_eA).verts(2)).world_coords; 
            vb1 = B2.verts(B2.edges(incr_eB).verts(1)).world_coords; 
            vb2 = B2.verts(B2.edges(incr_eB).verts(2)).world_coords; 
            normal = cross(va2-va1,vb2-vb1); 
            normCrossed = true;
            if norm(normal) > 0
               if (dot(B1.faces(B1.edges(incr_eA).faces(1)).normal + B1.faces(B1.edges(incr_eA).faces(2)).normal, normal) < 0)
                   normal = -normal;
               end
            else
                normal = ep2 - ep1;  
                normCrossed = false; 
            end
            
            %% OK, so here is our test if psi is negative
            if doEdgesPenetrate(B1,B2,incr_eA,incr_eB)
                if EDGE_DEBUG, disp('     EE: Edges in intersection'); end
                psi = -psi;   % There is edge penetration 
                if ~normCrossed
                    normal = -normal;   % The norm will have been initialized backwards
                end
            end
            
            % Throw out edges that don't cross other edges (both end points
            % project onto the same face).  
%             if ~does_EdgeCrossEdge(B1,B2,incr_eA,incr_eB)       %% TODO: This doesn't seem to work
%                if EDGE_DEBUG, disp('   EE: Edges don''t cross condition'); end
%                continue; 
%             end
            
            % Throw out edges that are on the opposite side of a body.  For
            % example, the tip of a tetrahedron (composed of three edges)
            % is near an edge.  The tetrahedron's edge that is farthest away
            % shouldn't make contact.  
            if ~valid_EdgeEdge( B1, B2, incr_eA, incr_eB, ep1, ep2 )
               if EDGE_DEBUG, disp('  EE: Invalid edge-edge'); end
               continue; 
            end
            
            % contact normal vector
            this_normal = normal/norm(normal);  % What's the very best way to define the edge normal?  Cross product? 

            % contact tangent vector
            this_tangent = arbitraryTangent(this_normal);  

            % Create contact (one subcontact per contact)
            sim.addContact(b1ID,b2ID,this_normal,this_tangent, ep1-B1.u, ep2-B2.u, psi);
  
        end
      end % for eB = 1:B2.num_edges
    end % for eA = 1: B1.num_edges
end % 
end % End EDGE_EDGE

% FIXME: do we need FACE-FACE collision?

end

