
% INPUT:
%   A   - A body 
%   B   - A body
% 
% OUTPUT:
%   C   - Set of all vertex-face contacts of A against B
function C = vertex_face_3d( A, B, eps_vf ) 
        
    C = []; 
    
    for va_index = 1:A.num_verts
       va = A.verts_world(va_index,:);
       for fb_index = 1:B.num_faces
          fb = B.faces(fb_index,:); 
          
          % FEASibility
          if feasibility_vf(A,B,va_index,fb_index,eps_vf)
              
              % APPlicability
              APPab = applicability_vf(A,B,va_index,fb_index);
              %if APPab > eps_app
                 
                  % Add contact
                  n = B.face_norms(fb_index,:);  
                  psi = dot( n, va - B.verts_world(fb(1),:));
                  c = Contact(A.bodyID, B.bodyID, va, -n, psi);
                  c.f1id = va_index;
                  c.f2id = fb_index;
                  c.applicability = APPab; 
                  c.type = 0;  % 1 => vertex-face
                  C = [C c]; 
              %end
          end
       end
    end

end

