
% INPUT:
%   A   - A body
%   B   - A body
%   ea  - Index of edge on A
%   eb  - Index of edge on B
%
% OUTPUT
%   orienation  - The applicability of ea on eb
%   n           - normal vector of a contact between ea and eb

function [orienation, n] = orientation_ee( A, B, ea, eb )

    % Edges
    Ea = A.edges(ea,:);
    Eb = B.edges(eb,:); 
    
    % Vertices
    Va1 = A.verts_world(Ea(1),:);
    Va2 = A.verts_world(Ea(2),:);
    Vb1 = B.verts_world(Eb(1),:);
    Vb2 = B.verts_world(Eb(2),:);

    % Face normals
    Fnb1 = B.face_norms(Eb(3),:);
    Fnb2 = B.face_norms(Eb(4),:);
    
    % Orientation vector of eb  (NOT normalized)
    Ob = cross3( Vb2-Vb1, -(Fnb1+Fnb2) );
    
    % Orientation of ea against eb
    orienation = dot3( Va2-Va1 , Ob); 
    
    % Normal 
    if orienation > 0
        n = cross3( Va2-Va1 , Vb2-Vb1 );
    else
        n = cross3( Vb2-Vb1 , Va2-Va1 );
    end
    n = n'/norm(n); 

end

