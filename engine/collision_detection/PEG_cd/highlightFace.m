

function highlightFace( B, f )

    c = rand(1,3);
    F = B.faces(f,:); 
    V1 = B.verts_world(F(1),:);
    V2 = B.verts_world(F(2),:);
    V3 = B.verts_world(F(3),:);
    plot3(V1(1),V1(2),V1(3),'*','Color',c);
    plot3(V2(1),V2(2),V2(3),'*','Color',c);
    plot3(V3(1),V3(2),V3(3),'*','Color',c);

end

