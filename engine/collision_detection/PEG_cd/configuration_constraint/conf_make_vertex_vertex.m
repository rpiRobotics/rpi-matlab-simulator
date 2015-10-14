

% INPUT:
%   A       - Body A
%   B       - Body B
%   va      - Index of vertex on A
%   vb      - Index of vertex on B
%   nc      - Number of contacts already found
%   eps_vf  - A vertex-face epsilon distance

function [ Contacts, Uconstraints, Iconstraints ] = conf_make_vertex_vertex( A,B,va,vb,nc,eps_vf )

    Contacts = [];
    Uconstraints = []; 
    Iconstraints = []; 
    
    % Gather adjacent features 
    ADJ_edges_va = get_adjacent_edges(A,va);
    ADJ_edges_vb = get_adjacent_edges(B,vb); 
    ADJ_faces_va = get_adjacent_faces(A,va);
    ADJ_faces_vb = get_adjacent_faces(B,vb);
    
    % Vertex-face feasibility and applicability
    FEAS_va_FB = zeros(1,length(ADJ_edges_vb));
    FEAS_vb_FA = zeros(1,length(ADJ_edges_va)); 
    APP_va_FB = FEAS_va_FB;
    APP_vb_FA = FEAS_vb_FA; 
    for fb_iter = 1:length(ADJ_faces_vb)
       FEAS_va_FB(fb_iter) = feasibility_vf(A,B,va,ADJ_faces_vb(fb_iter),eps_vf);
       APP_va_FB(fb_iter) = applicability_vf(A,B,va,ADJ_faces_vb(fb_iter));
    end
    for fa_iter = 1:length(ADJ_faces_va)
       FEAS_vb_FA(fa_iter) = feasibility_vf(B,A,vb,ADJ_faces_va(fa_iter),eps_vf);  
       APP_vb_FA(fa_iter) = applicability_vf(B,A,vb,ADJ_faces_va(fa_iter)); 
    end
    
    % Edge-edge feasibility and applicability
    FEAS_ea_eb = zeros(length(ADJ_edges_va), length(ADJ_edges_vb));
    APPL_ea_eb = FEAS_ea_eb; 
    for ea_iter = 1:length(ADJ_edges_va)
       for eb_iter = 1:length(ADJ_edges_vb)
          [FEAS_ea_eb(ea_iter,eb_iter), pa, pb, d]  = feasibility_ee(A,B,ADJ_edges_va(ea_iter),ADJ_edges_vb(eb_iter),eps_vf);
          % TODO: create temp ee contacts?
          
          APP_ea_eb(ea_iter,eb_iter) = applicability_ee(A,B,ADJ_edges_va(ea_iter),ADJ_edges_vb(eb_iter)); 
       end
    end
    
    % Use above information to determine potential contacts (FEAS and APPL)
    POT_va_FB = FEAS_va_FB & APP_va_FB > -.4; 
    POT_vb_FA = FEAS_vb_FA & APP_vb_FA > -.4;
    POT_ea_eb = FEAS_ea_eb & APP_ea_eb > -.4; 
    
    % Only a single viable vertex-contact
    if sum(POT_va_FB) == 1 || sum(POT_vb_FA) == 1
        % TODO: The edge-edge contacts accompanying the following vf contacts
        if sum(POT_va_FB) == 1
            % Unilateral constraint on vf contact TODO: is Uconst correct?
            c = conf_vertex_face(A,B,va,ADJ_faces_vb(POT_va_FB),eps_vf);
            Contacts = [Contacts c];
            Uconstraints = [Uconstraints constraint_unilateral(nc+length(Contacts))];
        end
        
        if sum(POT_vb_FA) == 1
            c = conf_vertex_face(B,A,vb,ADJ_faces_va(POT_vb_FA),eps_vf);
            Contacts = [Contacts c];
            Uconstraints = [Uconstraints constraint_unilateral(nc+length(Contacts))];
        end
    else 
        % Iconstraints for verts against faces
        % NOTE: There is a choice here, the consequences of which I'm not quite
        % sure about yet.  Consider a vertex va near 5 faces adjacent to vb,
        % where only 2 of these have APPL and FEAS.  Do we make the constraint
        % I(1,[2 3 4 5]) including all of the none APPL and none FEAS contacts,
        % or do we do I(1,2) where 1 and 2 were the contacts with APPL and FEAS
        if any(POT_va_FB)
           % Create contacts
           Fb = ADJ_faces_vb(POT_va_FB);
           C = [];
           for i=1:length(Fb)
              C = [C conf_vertex_face(A,B,va,Fb(i),eps_vf)]; 
           end
           primary_va_fb = peg_heuristic(C); 
           c_primary = C(primary_va_fb);        % Primary contact
           C(primary_va_fb) = [];               % Secondary contacts

           % Create Iconstraint
           Iconstraints = [Iconstraints constraint_inter_contact(nc+length(Contacts)+1,nc+length(Contacts)+2:nc+length(Contacts)+1+length(C))];
           Contacts = [Contacts c_primary C];
        end
        if any(POT_vb_FA) 
           % Create contacts
           Fa = ADJ_faces_va(POT_vb_FA);
           C = [];
           for i=1:length(Fa)
              C = [C conf_vertex_face(B,A,vb,Fa(i),eps_vf)]; 
              % The function calling expects contacts from A unto B, so we must "flip" the contact
           end
           primary_vb_fa = peg_heuristic(C); 
           c_primary = C(primary_vb_fa);        % Primary contact
           C(primary_vb_fa) = [];               % Secondary contacts

           % Create Iconstraint
           Iconstraints = [Iconstraints constraint_inter_contact(nc+length(Contacts)+1,nc+length(Contacts)+2:nc+length(Contacts)+1+length(C))];
           Contacts = [Contacts c_primary C];
        end
    end
    
    if ~any(POT_va_FB) && any(POT_vb_FA)
        
    end
    
    
    %% CASE 0: unilateral contacts 
    
    
    %% CASE 0: No vf have any applicability
    if ~any(FEAS_va_FB) && ~any(FEAS_vb_FA) 
       
        return;
    end
    
    %% CASE 1: ALL vf have applicability
    if all(FEAS_va_FB) && all(FEAS_vb_FA)
        % Iconstraint of vertices against faces
        % va on Fb
        C_va_Fb = conf_vertex_faces(A,B,va,eps_vf);  % ASSUMPTION!: the only faces within eps_vf will be those adjacent to vb
        % Determine primary contact
        
        % vb on Fa
        C_vb_Fa = conf_vertex_faces(B,A,vb,eps_vf); 
        
        
    
        return;  
    end % END CASE 1
    
    %% CASE 2: 
    if ~any(FEAS_va_FB)
        
        return; 
    end
    
    %% CASE 3:
    if ~any(FEAS_vb_FA)
       
        return;
    end
    
% 
% 
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
%       % Select edges of A that will have I-const with va-fb1
%       eaIconstFb1 = EaADJ(EadOrien > 0  &  EaAPP > -.05);
%       eaIconstFb2 = EaADJ(EadOrien < 0  &  EaAPP > -.05);

end

