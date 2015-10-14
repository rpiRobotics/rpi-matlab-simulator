

% INPUT:
%   A   - Body A
%   B   - Body B
%   va  - Index of vertex of A
%   eb  - Index of edge of B
%   pa  - 1x3 vector location of nearest point on A to eb
%   pb  - 1x3 vector location of nearest point on eb to A
%   num_contacts    - The number of contacts already found.  

function [C,U,I] = conf_make_vertex_edge( A,B,va,eb, pa,pb, num_contacts,eps_vf,eps_ee )

    nc = num_contacts; 
    C = []; 
    U = []; 
    I = []; 
    Va = A.verts_world(va,:); 
    Eb = B.edges(eb,:); 
                
    % Normals
    Vb1 = B.verts_world(Eb(1),:);
    Vb2 = B.verts_world(Eb(2),:);
    n1 = B.face_norms(Eb(3),:);
    n2 = B.face_norms(Eb(4),:); 

    % Case of edge dividing a plane.  Here, we ignore the ee contact and
    % only enforce vf contact, assuming there is no way for such an ee
    % contact to be violated if the vf is enforced.  
    if dot3(n1,n2) > .995
      %psi = dot(B.face_norms(Eb(3),:),pa-pb);
      %C = Contact(A.bodyID,B.bodyID,pa,-B.face_norms(Eb(3),:),psi);
      psi = dot(B.face_norms(Eb(3),:),Va-Vb1);
      C = Contact(A.bodyID,B.bodyID,Va,-B.face_norms(Eb(3),:),psi);  % TODO: use edge-edge instead?
      C.f1id = va;
      C.f2id = eb;
      C.type='ve'; 
      U = constraint_unilateral(num_contacts+1); 
      return;
    end
    %% Vertex-edge

    % orientation vector, and FEAS_vf 
    Ob = cross3( Vb2-Vb1, -(n1+n2) );  % Orientation vector of Eb
    Ob = Ob/norm(Ob); 
    FEAS_va_fb1 = feasibility_vf(A,B,va,Eb(3),eps_vf);
    FEAS_va_fb2 = feasibility_vf(A,B,va,Eb(4),eps_vf);

    %% The case that neither is feasible
    if ~FEAS_va_fb1 && ~FEAS_va_fb2
      % TODO: perhaps best is to put in unilateral edge-edge here?
    
%     %% The only vf contact can be with va fb1
%     elseif ~FEAS_va_fb2
%       % Unilateral constraint between va-fb1
%       c = Contact(A.bodyID,B.bodyID,Va,-n1,dot3(n1,Va-pb));
%       c.f1id = va;
%       c.f2id = Eb(3); 
%       c.type = 'vf';
%       C = c;
%       U = constraint_unilateral(nc+1);
%       
%       % Unilateral constraints on all applicable edges 
%       EaADJ = get_adjacent_edges(A,va);  % Adjacent edges to va
%       EaAPP = zeros(size(EaADJ));        % Applicability of adjacent edges on eb
%       EadOrien = EaAPP;                  % Dot product of each ea with eb's orientation vector
%       for ea_iter=1:length(EaADJ)
%          Ea = A.edges(EaADJ(ea_iter),:);
%          EaAPP(ea_iter) = applicability_ee(A,B,EaADJ(ea_iter),eb); 
%          if Ea(1) == va
%             ea_vec = A.verts_world(Ea(2),:) - Va;
%          else
%             ea_vec = A.verts_world(Ea(1),:) - Va;
%          end
%          ea_vec = ea_vec/norm(ea_vec); 
%          EadOrien(ea_iter) = dot3(Ob,ea_vec); 
%       end
%       eaIconstFb1 = EaADJ(EadOrien < 0  &  EaAPP > -.05);
%       for ea_iter=1:length(eaIconstFb1)
%           ea = eaIconstFb1(ea_iter);
%           [FEAS, pa, pb, ~] = feasibility_ee(A, B, ea, eb, eps_ee); 
%           if FEAS  % TODO: is this test needed? 
%             c = conf_make_edge_edge( A,B,ea,eb,pa,pb ); 
%             C = [C c]; 
%             U = [U constraint_unilateral(nc+length(C))];
%           end
%       end
%        
%     %% The only vf contact can be with va fb2
%     elseif ~FEAS_va_fb1
%       % Unilateral constraint between va-fb2
%       c = Contact(A.bodyID,B.bodyID,Va,-n2,dot3(n2,Va-pb));
%       c.f1id = va;
%       c.f2id = Eb(4); 
%       c.type = 'vf';
%       C = c;
%       U = constraint_unilateral(nc+1);
%       
%       % Unilateral constraints on all applicable edges 
%       EaADJ = get_adjacent_edges(A,va);  % Adjacent edges to va
%       EaAPP = zeros(size(EaADJ));        % Applicability of adjacent edges on eb
%       EadOrien = EaAPP;                  % Dot product of each ea with eb's orientation vector
%       for ea_iter=1:length(EaADJ)
%          Ea = A.edges(EaADJ(ea_iter),:);
%          EaAPP(ea_iter) = applicability_ee(A,B,EaADJ(ea_iter),eb); 
%          if Ea(1) == va
%             ea_vec = A.verts_world(Ea(2),:) - Va;
%          else
%             ea_vec = A.verts_world(Ea(1),:) - Va;
%          end
%          ea_vec = ea_vec/norm(ea_vec); 
%          EadOrien(ea_iter) = dot3(Ob,ea_vec); 
%       end
%       eaIconstFb2 = EaADJ(EadOrien > 0  &  EaAPP > -.05);
%       for ea_iter=1:length(eaIconstFb2)
%           ea = eaIconstFb2(ea_iter);
%           [FEAS, pa, pb, ~] = feasibility_ee(A, B, ea, eb, eps_ee); 
%           if FEAS  % TODO: is this test needed? 
%             c = conf_make_edge_edge( A,B,ea,eb,pa,pb ); 
%             C = [C c]; 
%             U = [U constraint_unilateral(nc+length(C))];
%           end
%       end
        
    %% va can contact either fb1 or fb2 
    else
      % I-constraint of Va against Fb1 and Fb2
      psi1 = dot3(n1,Va-pb);
      psi2 = dot3(n2,Va-pb); 
      c1 = Contact(A.bodyID,B.bodyID,Va,-n1,psi1);
      c1.f1id = va;
      c1.f2id = Eb(3);
      c1.type = 'vf';
      c2 = Contact(A.bodyID,B.bodyID,Va,-n2,psi2);
      c2.f1id = va;
      c2.f2id = Eb(4);
      c2.type = 'vf'; 

      C = [ c1 c2 ];                % The Vertex-Face contacts
      if c1.psi_n > -.005 && c1.psi_n < c2.psi_n  
        I = constraint_inter_contact(nc+1,nc+2);  
      elseif c2.psi_n > -.005
        I = constraint_inter_contact(nc+2,nc+1);
      else 
        if c1.psi_n > c2.psi_n
           I = constraint_inter_contact(nc+1,nc+2);
        else
           I = constraint_inter_contact(nc+2,nc+1);
        end
      end
%       if ~FEAS_va_fb1
%           I = constraint_inter_contact(nc+2,nc+1);
%       else
%           I = constraint_inter_contact(nc+1,nc+2); 
%       end

      % I-constraint for vf-ee pairs
      % Find edges of A connected to va with applicability to eb
      EaADJ = get_adjacent_edges(A,va);  % Adjacent edges to va
      EaAPP = zeros(size(EaADJ));        % Applicability of adjacent edges on eb
      EadOrien = EaAPP;                  % Dot product of each ea with eb's orientation vector
      for ea_iter=1:length(EaADJ)
         Ea = A.edges(EaADJ(ea_iter),:);
         EaAPP(ea_iter) = applicability_ee(A,B,EaADJ(ea_iter),eb); 
         if Ea(1) == va
            ea_vec = A.verts_world(Ea(2),:) - Va;
         else
            ea_vec = A.verts_world(Ea(1),:) - Va;
         end
         ea_vec = ea_vec/norm(ea_vec); 
         EadOrien(ea_iter) = dot3(Ob,ea_vec); 
      end
      % Select edges of A that will have I-const with va-fb1
      eaIconstFb1 = EaADJ(EadOrien > 0  &  EaAPP > -.1);
      eaIconstFb2 = EaADJ(EadOrien < 0  &  EaAPP > -.1);

      % Create contacts and I constraints
      for ea_iter=1:length(eaIconstFb1)
          ea = eaIconstFb1(ea_iter);
          [FEAS, pa, pb, ~] = feasibility_ee(A, B, ea, eb, eps_ee); 
          if FEAS_va_fb1
            c = conf_make_edge_edge( A,B,ea,eb,pa,pb ); 
            C = [C c]; 
            I = [I constraint_inter_contact(num_contacts+1,num_contacts+length(C))]; 
          elseif FEAS  % TODO: is this test needed? 
            c = conf_make_edge_edge( A,B,ea,eb,pa,pb ); 
            C = [C c]; 
            I = [I constraint_inter_contact(num_contacts+length(C),num_contacts+1)]; 
          end
      end
      for ea_iter=1:length(eaIconstFb2)
          ea = eaIconstFb2(ea_iter);
          [FEAS, pa, pb, ~] = feasibility_ee(A, B, ea, eb, eps_ee); 
          if FEAS_va_fb2
            c = conf_make_edge_edge( A,B,ea,eb,pa,pb ); 
            C = [C c]; 
            I = [I constraint_inter_contact(num_contacts+2,num_contacts+length(C))]; 
          elseif FEAS  % TODO: is this test needed? 
            c = conf_make_edge_edge( A,B,ea,eb,pa,pb ); 
            C = [C c]; 
            I = [I constraint_inter_contact(num_contacts+length(C),num_contacts+2)]; 
          end
      end
    end

     

end

















