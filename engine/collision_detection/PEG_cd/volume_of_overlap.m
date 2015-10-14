

% Given bodies A and B, determines the volume of their overlap V and 
% centroid of overlapping volume at point P.
function [V,P] = volume_of_overlap( A,B )

    % Set of points on or in the overlapping volume
    X = [];
    Y = [];
    Z = [];
    P = [0 0 0]; 
    
    % Edges of A against faces of B
    for ea_iter = 1:A.num_edges
      ea = A.edges(ea_iter,:);
      va1 = A.verts_world(ea(1),:);
      va2 = A.verts_world(ea(2),:);
      o = va1;
      d = va2-va1;
      d = d/norm(d); 
      for fb_iter = 1:B.num_faces  
        fb = B.faces(fb_iter,:);
        p0 = B.verts_world(fb(1),:);
        p1 = B.verts_world(fb(2),:);
        p2 = B.verts_world(fb(3),:);
        [flag, u, v, t] = rayTriangleIntersection (o, d, p0, p1, p2); 
        if flag && t>=0 && t <= norm(va2-va1)
           P = va1+t*d;
           X = [X; P(1)];
           Y = [Y; P(2)];
           Z = [Z; P(3)];
        end
      end
    end
    
    % Edges of B against faces of A
    for eb_iter = 1:B.num_edges
      eb = B.edges(eb_iter,:);
      vb1 = B.verts_world(eb(1),:);
      vb2 = B.verts_world(eb(2),:);
      o = vb1;
      d = vb2-vb1;
      d = d/norm(d); 
      for fa_iter = 1:A.num_faces  
        fa = A.faces(fa_iter,:);
        p0 = A.verts_world(fa(1),:);
        p1 = A.verts_world(fa(2),:);
        p2 = A.verts_world(fa(3),:);
        [flag, u, v, t] = rayTriangleIntersection (o, d, p0, p1, p2); 
        if flag && t>=0 && t <= norm(vb2-vb1)
           P = vb1+t*d;
           X = [X; P(1)];
           Y = [Y; P(2)];
           Z = [Z; P(3)];
        end
      end
    end
    
    for va_iter = 1:A.num_verts
        inside = true;
        for fb_iter = 1:B.num_faces
            c = conf_vertex_face(A,B,va_iter,fb_iter,100);
            if c.psi_n >= 0
               inside = false;
               break;
            end
        end
        if inside
            X = [X; A.verts_world(va_iter,1)];
            Y = [Y; A.verts_world(va_iter,2)];
            Z = [Z; A.verts_world(va_iter,3)];
        end
    end
    
    for vb_iter = 1:B.num_verts
        inside = true;
        for fa_iter = 1:A.num_faces
            c = conf_vertex_face(B,A,vb_iter,fa_iter,100);
            if c.psi_n >= 0
               inside = false;
               break;
            end
        end
        if inside
            X = [X; B.verts_world(vb_iter,1)];
            Y = [Y; B.verts_world(vb_iter,2)];
            Z = [Z; B.verts_world(vb_iter,3)];
        end
    end
    
    if ~isempty(X)
      try
        [K,V] = convhull(X,Y,Z);
        P = mean([X Y Z]); 
      catch
        V = 0;
      end
    else
      V = 0;
    end
    
%     for i=1:length(X)
%         plot3(X(i),Y(i),Z(i),'x');
%     end
    
    
end



















