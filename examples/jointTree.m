

% This is an attempt to recreate Jan Bender's tree as shown in
% http://i31www.ira.uka.de/~jbender/videos/tree127.mpg

function sim = jointTree( )

    %% Init simulator
    sim = Simulator();
    sim.H_dynamics = @mLCPdynamics;
    sim.H_collision_detection = []; 

    %% Create bodies
    ground = Body_plane([0;0;0],[0;0;1]);
    pole1 = mesh_cylinder(20,1,0.03,0.6);
        pole1.dynamic = false; 
        pole1.u = [0.6;0;0.3];
        pole1.color = [.7 .7 .7];
    pole2 = mesh_cylinder(20,1,0.03,.6);
        pole2.dynamic = false; 
        pole2.u = [0.3;0;0.6];
        pole2.quat = qt([0;1;0],pi/2); 
        pole2.color = [.7 .7 .7];
        
    r = 0.005;  % Radius of each cylindrical body
    nv = 5;     % Number of verts at each end of each cylinder
    dz = 0.05;   % Distance between each set of bars (yellow bars)
    dl = 0.6;   % Length of child bar relative to parent bar
    
    % First bar
    bar1 = mesh_cylinder(nv,0.5,r,0.5);
        bar1.u = [0;0;0.5]; 
        bar1.quat = qt([0;1;0],pi/2);
        bar1.color = [1;0;0]; 
        bar1 = body_updateMesh(bar1); 
    bars = bar1; 
    
    %% Returns two new bodies [b1 b2] with respect to the input body B
    function newBars = makeNextLevel(B)
        p1 = B.verts_world(1,:)';
        p2 = B.verts_world(nv+2,:)'; 
        b1 = mesh_cylinder(nv,B.mass/3,r,norm(p2-p1)*dl);
        b1.quat = qtmultiply(qt([0;0;1],pi/2),B.quat); % Rotate by pi/2 about Z
        b2 = b1;
        b1.u = p1 - [0;0;dz];
        b2.u = p2 - [0;0;dz]; 
        b1 = body_updateMesh(b1);
        b2 = body_updateMesh(b2);
        newBars = [b1 b2]; 
    end
    
    % Create multiple levels
    num_levels = 6; 
    barColor = eye(3); 
    for lvl = 1:num_levels
       tmpBars = [];
       for b=2^(lvl-1):2*(2^(lvl-1))-1
          tmpBars = [tmpBars  makeNextLevel(bars(b))]; 
          tmpBars(end-1).color = barColor(mod(lvl,3)+1,:); 
          tmpBars(end).color = barColor(mod(lvl,3)+1,:); 
          
          tmpBars(end-1).facealpha = 1;
          tmpBars(end).facealpha = 1; 
       end
       bars = [bars tmpBars]; 
    end
    
    %% Add bodies to simulator
    sim = sim_addBody( sim, [ground pole1 pole2 bars] );
    
    %% Add joints
    sim = sim_addJoint( sim, 3,4, bar1.u,[1;0;0],'spherical'); 
    bOff = 1; 
    for lvl = 1:num_levels
        for b=2^(lvl-1):2*(2^(lvl-1))-1
            %disp(['Connecting ' num2str(3+b) ' to ' num2str(3+b+bOff) ' and ' num2str(3+b+bOff+1) ]);
            sim = sim_addJoint( sim, 3+b,3+b+bOff, sim.bodies(3+b+bOff).u, [1;0;0],'spherical'); 
            bOff = bOff + 1; 
            sim = sim_addJoint( sim, 3+b,3+b+bOff, sim.bodies(3+b+bOff).u, [1;0;0],'spherical'); 
        end
    end
    
    %sim.drawJoints = true;
    sim.bodies(4).Fext(6) = 1; 
    
    %% User fuctnion 
    function S = userFunc( S )
        % Put yellow bars in between things
        if S.step == 1
            
            p01 = [0;0;.6];
            p1 = S.bodies(4).u; 
            plot3([p01(1) p1(1)],[p01(2) p1(2)],[p01(3) p1(3)],'y','linewidth',2);
            
            S.userData.yhandles = [];
            bodyOff = 1; 
            for L = 1:num_levels
                for b=2^(L-1):2*(2^(L-1))-1
                    b0 = 3+b;
                    b1 = 3+b+bodyOff;
                    b2 = 3+b+bodyOff+1;
                    
                    p01 = S.bodies(b0).verts_world(1,:)';
                    p02 = S.bodies(b0).verts_world(nv+2,:)'; 
                    
                    p1 = S.bodies(b1).u;
                    p2 = S.bodies(b2).u; 
                    
                    h1 = plot3([p01(1) p1(1)],[p01(2) p1(2)],[p01(3) p1(3)],'y','linewidth',1);
                    h2 = plot3([p02(1) p2(1)],[p02(2) p2(2)],[p02(3) p2(3)],'y','linewidth',1);
                    
                    S.userData.yhandles = [S.userData.yhandles h1 h2];
                    bodyOff = bodyOff + 1; 
                end
            end
        else
            bodyOff = 1; hi = 0;
            for L = 1:num_levels
                for b=2^(L-1):2*(2^(L-1))-1
                    b0 = 3+b;
                    b1 = 3+b+bodyOff;
                    b2 = 3+b+bodyOff+1;
                    
                    p01 = S.bodies(b0).verts_world(1,:)';
                    p02 = S.bodies(b0).verts_world(nv+2,:)'; 
                    
                    p1 = S.bodies(b1).u;
                    p2 = S.bodies(b2).u; 
                    
                    hi = hi+1; 
                    set(S.userData.yhandles(hi),'xdata',[p01(1) p1(1)]);
                    set(S.userData.yhandles(hi),'ydata',[p01(2) p1(2)]);
                    set(S.userData.yhandles(hi),'zdata',[p01(3) p1(3)]);
                    hi = hi+1;
                    set(S.userData.yhandles(hi),'xdata',[p02(1) p2(1)]);
                    set(S.userData.yhandles(hi),'ydata',[p02(2) p2(2)]);
                    set(S.userData.yhandles(hi),'zdata',[p02(3) p2(3)]);
                    
                    bodyOff = bodyOff + 1; 
                end
            end
        end
        
        if S.step < 300
           S.bodies(4).Fext(6) = 30;
        else
           S.bodies(4).Fext(6) = 0; 
        end
        
        %S.M(S.step) = getframe;
    end
    
    sim.userFunction = @userFunc;
    
    %% Run simulator
    sim.MAX_STEP = 500; 
    sim = sim_run( sim ); 
    
end




















