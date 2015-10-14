%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% doEdgesPenetrate.m 
%
% Determines if the edges e1 & e2 are in penetration. 
% Checks if e1 intersects more than one face of B2.  Similarly e2 on B1.
% The accuracy of this function relies entirely on the accuracy of the
% following statement:
%   If two edges are in penetration (psi < 0) then both of the following must be true: 
%       (1) Edge e1 of B1 intersects 2 faces of body B2, and at least one of those faces belongs to e2.
%       (2) Edge e2 of B2 intersects 2 faces of body B1, and at least one of those faces belongs to e1.

% There are many things that could be done to make this function faster...
function inPenetration = doEdgesPenetrate(B1,B2,e1,e2)
    
    inPenetration = false; 
    B1f1 = B1.edges(e1).faces(1);
    B1f2 = B1.edges(e1).faces(2);
    B2f1 = B2.edges(e2).faces(1);
    B2f2 = B2.edges(e2).faces(2);
    
    % Coordinate of first point on e1, & the vector from first to second point
    e1a = B1.verts(B1.edges(e1).verts(1)).world_coords';
    e1ab = B1.verts(B1.edges(e1).verts(2)).world_coords' - ...
            B1.verts(B1.edges(e1).verts(1)).world_coords';
    % Coordinate of first point on e2, & the vector from first to second point
    e2a = B2.verts(B2.edges(e2).verts(1)).world_coords';
    e2ab = B2.verts(B2.edges(e2).verts(2)).world_coords' - ...
            B2.verts(B2.edges(e2).verts(1)).world_coords'; 

    e1xB2 = 0;  % Count the # of times e1 intersects a face of B2.
    e2xB1 = 0;
    
    % First check the faces corresponding to e2 and e1, as they are the
    % most likely to be penetrated by each edge.  
    e1xB2f1 = does_edge_intersect_face( e1a, e1ab, ...
                                        B2.verts(B2.faces(B2.edges(e2).faces(1)).verts(1)).world_coords', ...
                                        B2.verts(B2.faces(B2.edges(e2).faces(1)).verts(2)).world_coords', ...
                                        B2.verts(B2.faces(B2.edges(e2).faces(1)).verts(3)).world_coords' );
    e1xB2f2 = does_edge_intersect_face( e1a, e1ab, ...
                                        B2.verts(B2.faces(B2.edges(e2).faces(2)).verts(1)).world_coords', ...
                                        B2.verts(B2.faces(B2.edges(e2).faces(2)).verts(2)).world_coords', ...
                                        B2.verts(B2.faces(B2.edges(e2).faces(2)).verts(3)).world_coords' );
    if e1xB2f1 && e1xB2f2
        inPenetration = true;
        return;
    elseif ~e1xB2f1 && ~e1xB2f2     % If neither face of B2 is intersected, psi must be positive. 
        inPenetration = false;
        return;
    end
    if e1xB2f1
        e1xB2 = e1xB2+1; 
    end
    if e1xB2f2
        e1xB2 = e1xB2+1; 
    end
    
    
    e2xB1f1 = does_edge_intersect_face( e2a, e2ab, ...
                                        B1.verts(B1.faces(B1.edges(e1).faces(1)).verts(1)).world_coords', ...
                                        B1.verts(B1.faces(B1.edges(e1).faces(1)).verts(2)).world_coords', ...
                                        B1.verts(B1.faces(B1.edges(e1).faces(1)).verts(3)).world_coords' );
    e2xB1f2 = does_edge_intersect_face( e2a, e2ab, ...
                                        B1.verts(B1.faces(B1.edges(e1).faces(2)).verts(1)).world_coords', ...
                                        B1.verts(B1.faces(B1.edges(e1).faces(2)).verts(2)).world_coords', ...
                                        B1.verts(B1.faces(B1.edges(e1).faces(2)).verts(3)).world_coords' ); 
    if e2xB1f1 && e2xB1f2
       inPenetration = true;
       return;
    elseif ~e2xB1f1 && ~e2xB1f2     % If neither face of B1 is intersected, psi must be positive. 
    end
    if e2xB1f1
       e2xB1 = e2xB1+1;
    end
    if e2xB1f2
       e2xB1 = e2xB1+1; 
    end
    
    % This is the part that really hurts computation time...  
    % Check remaining faces of B2 if necessary
    for f = 1:B2.num_faces
        if f == B2f1 || f == B2f2 
            continue;   % Avoid redundantly counting faces of e2
        end
       