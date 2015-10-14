
% OUTPUT:
%   C   - The set of edge-edge contacts
function C = edge_edge_3d( A, B, eps_ee )

    C = [];
    
    for ea_iter = 1:A.num_edges
       for eb_iter = 1:B.num_edges
          [FEAS, pa, pb, dist] = feasibility_ee(A, B, ea_iter, eb_iter, eps_ee); 
          if FEAS
              % Add contact
              %highlightEdge(A,ea_iter);
              %highlightEdge(B,eb_iter); 
              
              [orientationAB nAB] = orientation_ee(A,B,ea_iter,eb_iter);
              [orientationBA nBA] = orientation_ee(B,A,eb_iter,ea_iter);
              
              if sign(orientationAB) == sign(orientationBA)
                  n = nAB; 
                  psi = dot( n, pb-pa);
                  if psi > -0.1
                      c = Contact(A.bodyID, B.bodyID, pa, n, psi);
                      c.f1id = ea_iter;
                      c.f2id = eb_iter;
                      c.applicability = applicability_ee(A,B,ea_iter,eb_iter); 
                      c.type = 1;  % 1 => edge-edge
                      C = [C c]; 
                  end
              end
          end
       end
    end

end

