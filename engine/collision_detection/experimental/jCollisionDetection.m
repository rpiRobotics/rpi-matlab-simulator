

% Performs collision detection using rigid body constraints as described in 
% "Robot Motion Planning" by Jean-Claude Latombe

function jCollisionDetection( sim )

    % Loop over all bodies
    nb = length(sim.P);
    for i=1:nb-1
       bi = sim.P{i};
       for j=i+1:nb
            bj = sim.P{j};
           
            
            % Do not compare bodies that are set to not collide (e.g. joined bodies)
            if bi.doesNotCollideWith(bj.bodyID), continue; end
            
            % Don't collide two static bodies
            if (bi.static && bj.static), continue; end
            
            % Check bounding sphere.  
            U = bj.u - bi.u;
            D = norm(U);
            R = bi.bound + bj.bound;
            d = D-R;
            V = dot3(bi.nu(1:3)-bj.nu(1:3), U);
            dp = d - V*sim.h; 
            if dp < 0, continue; end
            clear U D R d V dp;
            
            
            %% Mesh - Mesh
            if strcmp(bi.body_type,'mesh') && strcmp(bj.body_type,'mesh')
                collide_trimesh_trimesh(sim, i,j, bi,bj);
           
            %% Sphere - Sphere
            elseif strcmp(bi.body_type,'sphere') && strcmp(bj.body_type,'sphere')
                n = bj.u-bi.u;                                     % Normal direction
                n = n/norm(n);                                     % Normalized
                t = arbitraryTangent(n);                           % Choose a tangent.
                p1 = n*bi.radius;                                  % [x;y;z] Position of contact on body_1 in world frame
                p2 = -n*bj.radius;                                 % [x;y;z] Position of contact on body_2 in world frame
                psi_n = norm(bj.u-bi.u) - bj.radius - bi.radius;   % Penetration depth
                sim.addContact(i,j,n,t,p1,p2,psi_n);
           
            %% Plane - Sphere
            elseif strcmp(bi.body_type,'plane') && strcmp(bj.body_type,'sphere')
                n = bi.n; 
                psi_n = dot( n, bj.u-bi.u ) - bj.radius;
                if psi_n < 0.2                  % TODO: Unfortunately, another hard-coded epsilon
                    t = arbitraryTangent(n); 
                    p2 = bj.u - bj.radius*n;
                    p1 = p2 - psi_n*n; 
                    sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                end
            end 
           
           
       end
    end

end



