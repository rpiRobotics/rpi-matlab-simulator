

% Simulation of WAM arm. 
function sim = wam( )
  
    %% Create simulator object
    sim = Simulator(0.002);
    sim.eps_vf = 0.003;
    sim.eps_ee = 0.003; 
    sim.H_collision_detection = @get_constraints_by_configuration; 
    sim.H_dynamics = @PEG3d;
    sim.userFunction = @wam_controller; 
    sim.MAX_STEP = 2100; 
    sim.drawContacts = true; 
    %sim.drawJoints = true; 
    sim.draw = true; 
    
    % Add bodies
    bodies = load('wam_bodies.mat'); 
    BODS = bodies.BODS; 
    
    POS = zeros(3,length(BODS));
    for b=1:length(BODS)
        POS(:,b) = BODS(b).u;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Fingers
    ROT_BODS = [    [0 0 sqrt(2)/2 sqrt(2)/2]                              % Palm
                    qtmultiply(qt([0;0;1],pi/2), qt([1;0;0],pi/2))'        % F1pivot
                    qtmultiply(qt([1;0;0],pi/2), qt([0;1;0],-pi/2))'       % F1a
                    qtmultiply(qt([0;1;0],pi/2), qt([0;0;1],-pi/2))'       % F1b
                    qtmultiply(qt([0;1;0],pi/2), qt([0;0;1],-pi/2))'       % F1c
                    qtmultiply(qt([0;1;0],pi), qt([0;0;1],pi/2))'          % F1d
                    qtmultiply(qt([0;0;1],-pi/2), qt([1;0;0],pi/2))'       % F2pivot
                    qtmultiply(qt([1;0;0],pi/2), qt([0;1;0],pi/2))'        % F2a
                    qtmultiply(qt([0;1;0],-pi/2), qt([0;0;1],pi/2))'       % F2b
                    qtmultiply(qt([0;1;0],-pi/2), qt([0;0;1],pi/2))'       % F2c
                    qtmultiply(qt([0;1;0],-pi), qt([0;0;1],-pi/2))'        % F2d
                    qtmultiply(qt([1;0;0],-pi/2),qt([0;0;1],pi))'          % F3a
                    qtmultiply(qt([1;0;0],pi/2),qt([0;0;1],pi))'           % F3b
                    qtmultiply(qt([1;0;0],pi/2),qt([0;0;1],pi))'           % F3c
                    qtmultiply(qt([0;1;0],pi),qt([0;0;1],pi))' ]';         % F3d
    
    %% FINGER 1
    ROT_OFFSET_F1 = -(pi/180) * [ 30             % Palm -> F1pivot
                                  0              % F1pivot -> F1a
                                 (90+37+4/9)     % F1a -> F1b
                                  84+2/9         % F1b -> F1c
                                  0     ];       % F1c -> F1d   
    ROT_VEC_F1 = [ 0 0 -1
                   0 0 1
                   0 -1 0
                   0 -1 0 
                   0 0 1 ]'; 
               
    % All body rotations, zero configuration
    finger_offset = 8;
    for b=1:length(BODS)-finger_offset
       BODS(b+finger_offset).quat = ROT_BODS(:,b);
    end
               
    % Body positions, finger 1
    QT = qt(ROT_VEC_F1(:,1), ROT_OFFSET_F1(1));
    BODS(finger_offset+3).u = BODS(finger_offset+2).u + qtrotate(QT,POS(:,finger_offset+3)-POS(:,finger_offset+2));
    
    BODS(finger_offset+4).u = BODS(finger_offset+3).u;
    
    QT = qtmultiply(qt(ROT_VEC_F1(:,3),ROT_OFFSET_F1(3)),QT);
    BODS(finger_offset+5).u = BODS(finger_offset+4).u + qtrotate(QT,POS(:,finger_offset+5)-POS(:,finger_offset+4)); 
    
    QT = qtmultiply(qt(ROT_VEC_F1(:,4),ROT_OFFSET_F1(4)),QT);
    BODS(finger_offset+6).u = BODS(finger_offset+5).u + qtrotate(QT,POS(:,finger_offset+6)-POS(:,finger_offset+5)); 
    
    
    %% Finger 2
    ROT_OFFSET_F2 = -(pi/180) * [ 30            % Palm -> F2pivot
                                  0             % F2pivot -> F2a
                                  2+4/9         % F2a -> F2b
                                  42+3/9            % F2b -> F2c
                                  0    ];       % F2c -> F2d
    ROT_VEC_F2 = [ 0 0 1
                   0 0 -1
                   0 1 0
                   0 1 0 
                   0 0 1 ]'; 
               
    % Body positions, finger 2
    QT = qt(ROT_VEC_F2(:,1), ROT_OFFSET_F2(1));
    BODS(finger_offset+8).u = BODS(finger_offset+7).u + qtrotate(QT,POS(:,finger_offset+8)-POS(:,finger_offset+7));
    
    BODS(finger_offset+9).u = BODS(finger_offset+8).u; 
    
    QT = qtmultiply(qt(ROT_VEC_F2(:,3),ROT_OFFSET_F2(3)),QT);
    BODS(finger_offset+10).u = BODS(finger_offset+9).u + qtrotate(QT,POS(:,finger_offset+10)-POS(:,finger_offset+9)); 
    
    QT = qtmultiply(qt(ROT_VEC_F2(:,4),ROT_OFFSET_F2(4)),QT);
    BODS(finger_offset+11).u = BODS(finger_offset+10).u + qtrotate(QT,POS(:,finger_offset+11)-POS(:,finger_offset+10)); 
    
    %% Finger 3
    ROT_OFFSET_F3 = -(pi/180) * [ 62+4/9
                                  0 
                                  62+4/9  ]; 
    ROT_VEC_F3 = [ -1 0 0
                   -1 0 0 
                   -1 0 0 ]'; 
               
    % Body positions, finger 3               
    QT = qt(ROT_VEC_F3(:,1), ROT_OFFSET_F3(1)); 
    BODS(finger_offset+14).u = BODS(finger_offset+13).u + qtrotate(QT,POS(:,finger_offset+14)-POS(:,finger_offset+13)); 
    
    QT = qtmultiply(qt(ROT_VEC_F3(:,3),ROT_OFFSET_F3(3)),QT);
    BODS(finger_offset+15).u = BODS(finger_offset+14).u + qtrotate(QT,POS(:,finger_offset+15)-POS(:,finger_offset+14)); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Update positions
    POS = zeros(3,length(BODS));
    for b=1:length(BODS)
        POS(:,b) = BODS(b).u;
    end
    
    BODS = BODS([(1:8)';9;11;12;14;16;17;19;20;21;23]);
    
    % The default COM for the WAM parts is where the joints go.  Let's improve this by assuming a geometric center of mass 
    for b=[1 3:length(BODS)];
       B = BODS(b);
       c = mean(B.verts_local(unique(convhull(B.verts_local)),:))';
       BODS(b).u = BODS(b).u + qtrotate(BODS(b).quat,c);
       for v=1:3
         BODS(b).verts_local(:,v) = BODS(b).verts_local(:,v) - c(v);
       end
    end
    sim = sim_addBody(sim,BODS); 
    
    % Change the mass of the bodies 
    BODS(2).mass = 4;   % Shoulder-1
    BODS(3).mass = 2;   % Shoulder-2
    BODS(4).mass = 2;   % Arm
    BODS(5).mass = 1;   % Elbow
    BODS(6).mass = 1;   % Forearm
    BODS(7).mass = 1;   % Wrist-1
    BODS(8).mass = 1;   % Wrist-2
    BODS(9).mass = 1;   % Palm
    finger_mass = 0.2; 
    BODS(10).mass = finger_mass;   % F1a
    BODS(11).mass = finger_mass;   % F1b
    BODS(12).mass = finger_mass;   % F1c
    BODS(13).mass = finger_mass;   % F2a
    BODS(14).mass = finger_mass;   % F2b
    BODS(15).mass = finger_mass;   % F2c
    BODS(16).mass = finger_mass;   % F3a
    BODS(17).mass = finger_mass;   % F3b
    BODS(18).mass = finger_mass;   % F3c
    
    %% Add joints
    % Arm joints
    sim = sim_addJoint(sim,1,2,sim.bodies(2).u,[0;0;1],'revolute');     % Base-shoulder
    sim = sim_addJoint(sim,2,3,POS(:,3),[-1;0;0],'revolute');            % Shoulder-shoulder2
    sim = sim_addJoint(sim,3,4,POS(:,4),[0;0;1],'revolute');            % Shoulder2-arm
    sim = sim_addJoint(sim,4,5,POS(:,5),[-1;0;0],'revolute');            % arm-elbow
    sim = sim_addJoint(sim,5,6,POS(:,6),[0;0;1],'fixed');               % elbow-forearm
    sim = sim_addJoint(sim,6,7,POS(:,7),[0;0;1],'fixed');               % forearm-wrist
    sim = sim_addJoint(sim,7,8,POS(:,7),[-1;0;0],'revolute');               % wrist-wrist
    sim = sim_addJoint(sim,8,9,POS(:,9),[0;0;1],'fixed');               % wrist-palm
    
    % Hand joints
    % Finger 1
    sim = sim_addJoint(sim,finger_offset+1,finger_offset+2,POS(:,finger_offset+2),[0;0;1],'revolute');      % Palm->F1a
    sim = sim_addJoint(sim,finger_offset+2,finger_offset+3,POS(:,finger_offset+3),[0;-1;0],'revolute');     % F1a->F1b
    sim = sim_addJoint(sim,finger_offset+3,finger_offset+4,POS(:,finger_offset+5),[0;-1;0],'revolute');     % F1b->F1d
    % Finger 2
    sim = sim_addJoint(sim,finger_offset+1,finger_offset+5,POS(:,finger_offset+7),[0;0;1],'revolute');      % Palm->F2a
    sim = sim_addJoint(sim,finger_offset+5,finger_offset+6,POS(:,finger_offset+8),[0;1;0],'revolute');      % F2a->F2b
    sim = sim_addJoint(sim,finger_offset+6,finger_offset+7,POS(:,finger_offset+10),[0;1;0],'revolute');     % F2b->F2d
    % Finger 3
    sim = sim_addJoint(sim,finger_offset+1,finger_offset+8,POS(:,finger_offset+12),[0;0;1],'fixed');        % Palm->F2a
    sim = sim_addJoint(sim,finger_offset+8,finger_offset+9,POS(:,finger_offset+12),[-1;0;0],'revolute');    % F3a->F3b
    sim = sim_addJoint(sim,finger_offset+9,finger_offset+10,POS(:,finger_offset+14),[-1;0;0],'revolute');   % F3b->F3d
    
    % Set non-collidable for most bodies
    for b=1:6
       sim.bodies(b).collides = false;  
    end
    for b=6:length(BODS)
       %sim.bodies(b).collides = false;  
       sim.bodies(b).doesNotCollideWith = 1:length(BODS); 
       sim.bodies(b).facealpha = 0.9; 
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add table 
    table = mesh_rectangularBlock(2.1495,1.067,0.03); %.01905); 
        %table.u = [0;0.76;0.13];
        table.u = [0;0.76;0.25];
        table.dynamic = false;
        table.color = [.9 .9 .9];
        table.facealpha = 0.9;
    sim = sim_addBody(sim,table); 
    
    % Add object
    ico = mesh_icosahedron();
        ico = scale_mesh(ico,.04);
        ico.J = 0.0001*eye(3); 
        ico.u = [0.0;0.68;.38];
        ico.quat = qt([1;2;3],pi/7); 
        ico.color = [.8 .3 .4];
        ico.facealpha = 1;
    sim = sim_addBody(sim,ico); 
    
    
    %% Run simulation 
    sim = sim_run(sim);

end



















