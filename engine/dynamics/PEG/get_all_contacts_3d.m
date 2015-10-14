

% Given a Simulation object sim, finds all potential vertex-face contacts
% and edge-edge contacts between every pair of bodies.  

function sim = get_all_contacts_3d( sim )

    
    eps_vf = .4; 
    eps_ee = .3; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN function 
    %% Clear previous contacts
    C = [];
    [sim.bodies.active] = deal(false);  
    sim.num_activeBodies = 0; 
    
          % TODO: This is already done in preDynamics()
          nj = length(sim.joints); 
          for j=1:nj
             sim = sim_activateBodies( sim, sim.joints(j).body1id, sim.joints(j).body2id );
          end
          njc = sim.num_jointConstraints; 

    %% Iterate over body pairs 
    num_bodies = length(sim.bodies);
    for Aid = 1:num_bodies-1 
        A = sim.bodies(Aid);
        for Bid = Aid+1:num_bodies
            B = sim.bodies(Bid); 
            
            % Don't collide static bodies
            % Also, some bodies are listed as "doNotCollide" with each other,
            %        e.g. bodies with joints that require overlap.
            %% Broad-phase test
            if ~A.dynamic && ~B.dynamic || any(A.doesNotCollideWith == B.bodyID) || ~cd_intersect_AABBs(A,B)
                continue; 
            end   
            
            % Vertex-face
            Ca = vertex_face_3d(A,B,eps_vf);
            Cb = vertex_face_3d(B,A,eps_vf); 
            if ~isempty(Ca) || ~isempty(Cb)
                C = [ C Ca Cb ];
                sim = sim_activateBodies( sim, Aid, Bid ); 
            end 
            
            sim.userData.numVertContacts = length(C); 
            
            % Edge-edge
            Cee = edge_edge_3d(A,B,eps_ee);  
            if ~isempty(Cee)
                C = [ C Cee ];
                sim = sim_activateBodies( sim, Aid, Bid ); 
            end 
       end
    end
    
    sim.contacts = C; 

end













