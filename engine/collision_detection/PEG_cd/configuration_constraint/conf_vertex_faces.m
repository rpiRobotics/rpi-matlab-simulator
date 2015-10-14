
% Contact identification for only 1 vertex va against all faces of B
function C = conf_vertex_faces( A,B, va_index, eps_vf )

   C = [];
   va = A.verts_world(va_index,:); 
   for fb_index = 1:B.num_faces
      fb = B.faces(fb_index,:); 

      % FEASibility
      if feasibility_vf(A,B,va_index,fb_index,eps_vf)

          % APPlicability
          APPab = applicability_vf(A,B,va_index,fb_index);
          if APPab > -.3
              % Add contact
              n = B.face_norms(fb_index,:);  
              psi = dot3( n, va - B.verts_world(fb(1),:));
              c = Contact(A.bodyID, B.bodyID, va, -n, psi);
              c.f1id = va_index;
              c.f2id = fb_index;
              %c.applicability = APPab; 
              c.type = 'vf';  % 1 => vertex-face
              C = [C c]; 
          end
      end
   end

end

