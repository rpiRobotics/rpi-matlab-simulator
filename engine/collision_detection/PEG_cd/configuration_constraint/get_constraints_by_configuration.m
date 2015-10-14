

function sim = get_constraints_by_configuration( sim )

    %eps_vf = 10*sim.h; %.1; 
    %eps_ee = 10*sim.h; %.1; 
    
    %eps_vf = 0.03;
    %eps_ee = 0.03;
    eps_vf = sim.eps_vf;
    eps_ee = sim.eps_ee; 
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN function 
    %% Clear previous contacts
    C = [];
    Uconstraints = [];
    Iconstraints = [];
    [sim.bodies.active] = deal(false);  
    sim.num_activeBodies = 0; 
    
          % TODO: This is already done in preDynamics()
%           nj = length(sim.joints); 
%           for j=1:nj
%              sim = sim_activateBodies( sim, sim.joints(j).body1id, sim.joints(j).body2id );
%           end
%           njc = sim.num_jointConstraints; 

    %% Iterate over body pairs 
    num_bodies = length(sim.bodies);
    for Aid = 1:num_bodies-1 
        A = sim.bodies(Aid);
        for Bid = Aid+1:num_bodies
            B = sim.bodies(Bid); 

            % Broad-phase test
            if ~A.dynamic && ~B.dynamic || any(A.doesNotCollideWith == B.bodyID) || ...
                    ~A.collides || ~B.collides || ~cd_intersect_AABBs(A,B)
                continue; 
            end   
            
            % Configuration dependent contacts and constraints 
            [c, u, i] = configuration_determine_constraints(A,B,eps_vf,eps_ee);
            if ~isempty(c)
               sim = sim_activateBodies( sim, Aid, Bid );
               % Offset the contact indices of the constraints
               for u_iter = 1:length(u)
                   u(u_iter).C = u(u_iter).C + length(C);
               end
               for i_iter = 1:length(i)
                   for iC1_iter = 1:length(i(i_iter).C1)
                      i(i_iter).C1(iC1_iter) = i(i_iter).C1(iC1_iter) + length(C);
                   end
                   for iC2_iter = 1:length(i(i_iter).C2)
                      i(i_iter).C2(iC2_iter) = i(i_iter).C2(iC2_iter) + length(C);
                   end
               end
               C = [C c];
               Uconstraints = [Uconstraints u];
               Iconstraints = [Iconstraints i]; 
            end
            
       end
    end
    
    sim.contacts = C; 
    sim.Uconstraints = Uconstraints;
    sim.Iconstraints = Iconstraints; 



    
end

