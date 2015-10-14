

function sim = PEG3d( sim )


    if isempty(sim.contacts) && sim.num_jointConstraints <= 0
        newNU = []; return; 
    end

    %% Initialize matrices
    
    % Useful vars 
    C = sim.contacts; 
    nb = sim.num_activeBodies;    % Number of bodies with contacts

    % Init submatrices 
    M = sim.dynamics.M;
    NU = sim.dynamics.NU;
    FX = sim.dynamics.FX; 
    
    
    %% Unilateral constraints
    Gn = sparse( 6*nb, length(sim.Uconstraints) ); 
    psi_n = zeros( length(sim.Uconstraints), 1 );
    
    U = sim.Uconstraints;
    for i = 1:length(U)
        ci = C(U(i).C);   % ith contact  
        
        B1 = sim.bodies(ci.body1_id);
        B2 = sim.bodies(ci.body2_id);
        
        % Body 1 portion
        if B1.dynamic
            Gn(6*B1.bodyContactID-5:6*B1.bodyContactID, i) = [ -ci.normal'
                                        cross3( ci.p1' - B1.u, -ci.normal' ) ];
        end
        
        % Body 2 portion
        if B2.dynamic
            Gn (6*B2.bodyContactID-5:6*B2.bodyContactID, i) = [ ci.normal'
                                         cross3( (ci.p1' + ci.psi_n*ci.normal') - B2.u, ci.normal' ) ];
        end
        
        psi_n(i) = ci.psi_n; 
        
    end
    
    
    %% Inter-contact constraints
    % In 2D, we have the nice feature that all I-constraints have exactly 
    % 2 contacts.  We will utilize this throughout. 
    I = sim.Iconstraints; 
    nc = length(I);
    ns = 0;                     % Count number of secondary contacts
    for i=1:length(I)
       ns = ns + 1 + length(I(i).C2); 
    end
    Gc = sparse( 6*nb, ns-nc ); 
    Gp = sparse( 6*nb, nc );   % Only primary contacts go in Gp
    psi_c = zeros( ns-nc, 1 );
    psi_p = zeros( nc, 1 );    % Again, only primary contacts  
    
 
    Epc = sparse(nc,ns-nc);
    Ec = sparse(ns-nc, ns-nc); 
    
    gc_offset = 1; 
    for i = 1:length(I)
        gc_offset_start = gc_offset; 
        
        c1 = C(I(i).C1(1));     % With I-constraints, there is at least one iota contact
        if isempty(I(i).C2)  
           C2 = C(I(i).C1(2));
        else
           C2 = C(I(i).C2);  
        end
        
        % IMPORTANT!  It is necessary that c2 is in the same frame as c1,
        % therefore, if c1 is from body A->B and c2 is from body B->A, we
        % must "flip" c2 to match c1.
        for c2_iter = 1:length(C2)
            c2 = C2(c2_iter); 
            if c1.body1_id ~= c2.body1_id
                c_temp = c2;
                c2.body1_id = c_temp.body2_id;
                c2.body2_id = c_temp.body1_id;
                c2.p1 = c_temp.p1 + c_temp.psi_n * c_temp.normal;
                c2.normal = -c_temp.normal;
                c2.f1id = c_temp.f2id;
                c2.f2id = c_temp.f1id;
            end
            C2(c2_iter) = c2; 
        end
        
        B1 = sim.bodies(c1.body1_id);
        B2 = sim.bodies(c1.body2_id);
        
        % Jacobian of primary contact 
        G1a = [ -c1.normal'
                cross3( c1.p1' - B1.u, -c1.normal' ) ];
        G1b = [ c1.normal'
                cross3( (c1.p1' + c1.psi_n*c1.normal') - B2.u, c1.normal' ) ]; 
        
            
        % Gp -- Primary contact
        if B1.dynamic
            Gp(6*B1.bodyContactID-5:6*B1.bodyContactID, i) = G1a; 
        end
        if B2.dynamic
            Gp (6*B2.bodyContactID-5:6*B2.bodyContactID, i) = G1b;  
        end
        psi_p(i) = c1.psi_n;     
           
        
        % Gc -- Secondary contacts
        for c2_iter = 1:length(C2) 
            c2 = C2(c2_iter); 
            G2a = [ -c2.normal'
                    cross3( c2.p1' - B1.u, -c2.normal' ) ];
            G2b = [ c2.normal'
                    cross3( (c2.p1' + c2.psi_n*c2.normal') - B2.u, c2.normal' ) ]; 

            % Gc
            if B1.dynamic
                Gc(6*B1.bodyContactID-5:6*B1.bodyContactID, gc_offset) = G1a-G2a;
            end
            if B2.dynamic
                Gc (6*B2.bodyContactID-5:6*B2.bodyContactID, gc_offset) = G1b-G2b;
            end
            psi_c(i+gc_offset-gc_offset_start) = c1.psi_n - c2.psi_n; 
            
            gc_offset = gc_offset + 1; 
        end
        
        % Ep and Epc
        Epc(i,gc_offset_start:gc_offset-1) = 1;  % TODO: something's wrong in the offsets
        Ec(gc_offset_start:gc_offset-1,gc_offset_start:gc_offset-1) = tril(ones(gc_offset-gc_offset_start));

    end
    
    nu = length(U); 
    nc = size(Gc,2);
    np = size(Gp,2);
    Gb = sim.dynamics.Gb; 
    njc = sim.num_jointConstraints; 

    %% Construct PEG formulation
    A = [ -M   Gb     Gn           Gp      zeros(6*nb, nc)
          Gb'  zeros(njc,njc+nu+np+nc)
          Gn'  zeros(nu,njc+nu+np+nc)
          Gp'  zeros(np,njc+nu+np)         Epc                 
          Gc'  zeros(nc,njc+nu+np)         Ec       ];  
      
    h = sim.h;
    b = [ M*NU + FX*h
          sim.dynamics.joint_bn
          psi_n/h
          psi_p/h 
          psi_c/h ] ;  
      
      
    % Solve the PEG MLCP  
    problem_size = size(A,1);
    z0 = zeros(problem_size,1);       
    big=10^20;    
    u = big*ones(problem_size,1);
    l = zeros(size(u));
    l(1:6*nb+njc) = -big;

    try 
        tic;
        newNU = pathlcp(A,b,l,u,z0);
        solveTime = toc; 
    catch
        try
            % Initialize z0 with current body velocities 
            for bid=1:length(sim.bodies)
               B = sim.bodies(bid);
               if B.dynamic && B.active, z0(6*B.bodyContactID-5:6*B.bodyContactID) = B.nu; end
            end 

            newNU = pathlcp(A,b,l,u,z0);
            disp('Path failed with z0 of zeros.  Retrying with initialized z0...');
        catch
            disp('Path failed with z0 of zeros and init z0....');
           save('saved_sim_failed.mat','sim');
        end
    end
    
    sim.userData.problemSize(sim.step) = size(A,1);
    sim.userData.solveTime(sim.step) = solveTime;
    sim.newNU = newNU; 

end

