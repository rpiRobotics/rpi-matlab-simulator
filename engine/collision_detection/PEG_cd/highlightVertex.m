
% INPUT: 
%   B   - Body
%   v   - Vertex index

function highlightVertex( B, v )

    V = B.verts_world(v,:);
    plot3(V(1),V(2),V(3),'*','Color',rand(1,3));

end

