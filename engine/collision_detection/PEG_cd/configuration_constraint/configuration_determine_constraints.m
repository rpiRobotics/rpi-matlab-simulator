
% Returns the set of constraints, based on configuration of A and B
function [Contacts, Uconstraints, Iconstraints] = configuration_determine_constraints( A,B,eps_vf,eps_ee )

    Contacts = []; 
    Uconstraints = [];
    Iconstraints = [];
    
    
    function addContacts(C,U,I)
       if isempty(C)
           return;
       end
       
       % For each unilateral constraint, only add after verifying there 
       % doesn't already exist a U constraint for it.
       %toRemove = [];
       for u_iter=1:length(U)
           c = C(U(u_iter).C-length(Contacts));  
           if strcmp(c.type,'vf')
               if c.body1_id == A.bodyID && ~VertexFaceHasUconstraint_AB(c.f1id,c.f2id) 
                   VertexFaceHasUconstraint_AB(c.f1id,c.f2id) = 1;
                   Uconstraints = [Uconstraints U(u_iter)]; 
               elseif c.body1_id == B.bodyID && ~VertexFaceHasUconstraint_BA(c.f1id,c.f2id)
                   VertexFaceHasUconstraint_BA(c.f1id,c.f2id) = 1;
                   Uconstraints = [Uconstraints U(u_iter)]; 
               else
                   %toRemove = [toRemove u_iter];
               end
           else
               if c.body1_id == A.bodyID 
                   b_edge = B.edges(c.f2id,:);
                   VertexFaceHasUconstraint_AB(c.f1id,b_edge(3)) = 1;
                   VertexFaceHasUconstraint_AB(c.f1id,b_edge(4)) = 1;
               elseif c.body1_id == B.bodyID 
                   a_edge = A.edges(c.f2id,:);
                   VertexFaceHasUconstraint_BA(c.f1id,a_edge(3)) = 1;
                   VertexFaceHasUconstraint_BA(c.f1id,a_edge(4)) = 1;
               end
           end
       end
       
       % Remove contacts for unilateral constraints that were removed.
       %C(toRemove) = [];
       
       % Assert that all contacts are in frame A unto B
       for c_iter=1:length(C)
           c = C(c_iter);
           if c.body1_id ~= A.bodyID 
              c_temp = c;
              c.body1_id = c_temp.body2_id;
              c.body2_id = c_temp.body1_id;
              c.p1 = c_temp.p1 + c_temp.psi_n * c_temp.normal;
              c.normal = -c_temp.normal;
              c.f1id = c_temp.f2id;
              c.f2id = c_temp.f1id;
              C(c_iter) = c; 
           end
       end
       
       % Add all contacts and Iconstraints
       Contacts = [Contacts C];
       Uconstraints = [Uconstraints U]; 
       Iconstraints = [Iconstraints I]; 
    end

    
    % To avoid redundant contacts (and save time), we want to track which
    % features of A and B already have contacts with the other body.  
    Va_has_contact = sparse(A.num_verts,1);
    %Ea_has_contact = zeros(A.num_edges,1);
    Vb_has_contact = sparse(B.num_verts,1);
    %Eb_has_contact = zeros(B.num_edges,1);
    
    %sparseSize = 10; %round(sqrt(A.num_verts));
    VertexVertexHasContact = sparse(A.num_verts,B.num_verts); 
    VertexEdgeHasContact_AB = sparse(A.num_verts,B.num_edges);
    VertexEdgeHasContact_BA = sparse(B.num_verts,A.num_edges); 
    VertexFaceHasUconstraint_AB = sparse(A.num_verts,B.num_faces);
    VertexFaceHasUconstraint_BA = sparse(B.num_verts,A.num_faces); 
    %EdgeEdgeHasIconstraint = sparse(A.num_edges,B.num_edges,sparseSize);
    %EdgeEdgeTODO = sparse(A.num_edges,B.num_edges,sparseSize);
    EdgeEdgeTODO = []; 
    
    
    % Because vertices are a subset of edges, we will discover all
    % potential contacts of vf and ee by looping over edges 
    for ea_iter = 1:A.num_edges
      Ea = A.edges(ea_iter,:);
      va1 = A.verts_world(Ea(1),:);
      va2 = A.verts_world(Ea(2),:); 
      
      for eb_iter = 1:B.num_edges
        Eb = B.edges(eb_iter,:);
        vb1 = B.verts_world(Eb(1),:);
        vb2 = B.verts_world(Eb(2),:); 
        
        [FEAS, pa, pb, dist] = feasibility_ee(A, B, ea_iter, eb_iter, eps_ee); 
        % If the edge segment is not within eps_ee, then we assume the
        % vertices that are at the extents of the edges are not within
        % eps_vf, and no longer consider contacts here
        if FEAS
           va1dist = norm(va1-pa);
           va2dist = norm(va2-pa);
           vb1dist = norm(vb1-pb);
           vb2dist = norm(vb2-pb);
           
           %% Vertex-edge and vertex-vertex possibilities
           if va1dist <= eps_vf
              Va_has_contact(Ea(1)) = 1; 
              % a1,b1
              if vb1dist <= eps_vf
                if ~VertexVertexHasContact(Ea(1),Eb(1))
                    VertexVertexHasContact(Ea(1),Eb(1)) = 1;
                    Vb_has_contact(Eb(1)) = 1; 
                    [C,U,I] = conf_make_vertex_vertex(A,B,Ea(1),Eb(1),length(Contacts),eps_vf);
                    addContacts(C,U,I);
                end
              % a1,b2
              elseif vb2dist <= eps_vf  % TODO: these are not actually mutually exclusive, this assumes |ea| > eps_ee
                if ~VertexVertexHasContact(Ea(1),Eb(2))
                    VertexVertexHasContact(Ea(1),Eb(2)) = 1;
                    Vb_has_contact(Eb(2)) = 1; 
                    [C,U,I] = conf_make_vertex_vertex(A,B,Ea(1),Eb(2),length(Contacts),eps_vf);
                    addContacts(C,U,I);
                end
              % a1,eb
              else
                if ~VertexEdgeHasContact_AB(Ea(1),eb_iter)
                    VertexEdgeHasContact_AB(Ea(1),eb_iter) = 1;
                    [C,U,I] = conf_make_vertex_edge(A,B,Ea(1),eb_iter,pa,pb,length(Contacts),eps_vf,eps_ee);
                    addContacts(C,U,I);
                end
              end
           elseif va2dist <= eps_vf  
              Va_has_contact(Ea(2)) = 1; 
              % a2,b1
              if vb1dist <= eps_vf
                if ~VertexVertexHasContact(Ea(2),Eb(1))
                    VertexVertexHasContact(Ea(2),Eb(1)) = 1; 
                    Vb_has_contact(Eb(1)) = 1; 
                    [C,U,I] = conf_make_vertex_vertex(A,B,Ea(2),Eb(1),length(Contacts),eps_vf);
                    addContacts(C,U,I);
                end
              % a2,b2
              elseif vb2dist <= eps_vf  
                if ~VertexVertexHasContact(Ea(2),Eb(2))
                    VertexVertexHasContact(Ea(2),Eb(2)) = 1; 
                    Vb_has_contact(Eb(2)) = 1; 
                    [C,U,I] = conf_make_vertex_vertex(A,B,Ea(2),Eb(2),length(Contacts),eps_vf);
                    addContacts(C,U,I);
                end
              % a2,eb
              else
                if ~VertexEdgeHasContact_AB(Ea(2),eb_iter)
                    VertexEdgeHasContact_AB(Ea(2),eb_iter) = 1;
                    [C,U,I] = conf_make_vertex_edge(A,B,Ea(2),eb_iter,pa,pb,length(Contacts),eps_vf,eps_ee);
                    addContacts(C,U,I);
                end
              end
           % b1,ea
           elseif vb1dist <= eps_vf
              if ~VertexEdgeHasContact_BA(Eb(1),ea_iter)
                  VertexEdgeHasContact_BA(Eb(1),ea_iter) = 1; 
                  [C,U,I] = conf_make_vertex_edge(B,A,Eb(1),ea_iter,pb,pa,length(Contacts),eps_vf,eps_ee);
                  addContacts(C,U,I);
              end
           % b2,ea
           elseif vb2dist <= eps_vf 
              if ~VertexEdgeHasContact_BA(Eb(2),ea_iter)
                  VertexEdgeHasContact_BA(Eb(2),ea_iter) = 1; 
                  [C,U,I] = conf_make_vertex_edge(B,A,Eb(2),ea_iter,pb,pa,length(Contacts),eps_vf,eps_ee);
                  addContacts(C,U,I);
              end
           % edge-edge contact
           else
              EdgeEdgeTODO = [EdgeEdgeTODO; ea_iter eb_iter pa pb];  % Store info for possible use later
           end
        end % END if FEAS 
      end % END for eb_iter
    end % END for ea_iter
    
    
    %% AFTER all of the I(and X)-constraints are identified, we complete 
    % the U-constraints.  Here, we go through edge-edge. 
    for ee_iter = 1:size(EdgeEdgeTODO,1)
        % Make sure there is not a redundant ea-eb contact in the I-constraints
        ea_iter = EdgeEdgeTODO(ee_iter,1);
        eb_iter = EdgeEdgeTODO(ee_iter,2);
        moveOn = false;
        for ci=1:length(Contacts)
           if Contacts(ci).f1id == ea_iter && Contacts(ci).f2id == eb_iter && strcmp(Contacts(ci).type,'ee')
              moveOn = true;
              break;
           end
        end
        if moveOn, continue; end
        
        pa = EdgeEdgeTODO(ee_iter,3:5);
        pb = EdgeEdgeTODO(ee_iter,6:8);
        C = conf_make_edge_edge(A,B,ea_iter,eb_iter,pa,pb); 
        if ~isempty(C)
         Contacts = [Contacts C];
         Uconstraints = [Uconstraints constraint_unilateral(length(Contacts))]; 
        end
    end
    
    
    %% Possibility of vf contact if no other Ea contact
    for va=1:A.num_verts
    % Check we haven't already found a contact for each vertex
      if ~Va_has_contact(va)
         C = conf_vertex_faces(A,B,va,eps_vf);
         if ~isempty(C)
            c = C(peg_heuristic(C)); 
            c.type = 'vf';
            Contacts = [Contacts c];
            Uconstraints = [Uconstraints constraint_unilateral(length(Contacts))];
         end
         Va_has_contact(va) = 1; 
      end
    end
    for vb = 1:B.num_verts
      if ~Vb_has_contact(vb) 
         C = conf_vertex_faces(B,A,vb,eps_vf);
         if ~isempty(C)
            c = C(peg_heuristic(C)); 
            c.type = 'vf';
            Contacts = [Contacts c];
            Uconstraints = [Uconstraints constraint_unilateral(length(Contacts))];
         end
         Vb_has_contact(vb) = 1;
      end
    end
    
end

