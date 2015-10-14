


% Uses Bullet collision detection. 
% Currently only works for mesh bodies. 
function sim = BulletCD( sim )

    % Clear contact constraints
    C = [];
    [sim.bodies.active] = deal(false);  % TODO: test if this is actually more efficient 
    sim.num_activeBodies = 0; 
    
    % Fields that will be passed to Bullet
    NB = sim.num_bodies; 
    bID = zeros(NB,1);
    VERT_offset = zeros(NB,1);  
    VERTS = [];             % zeros(3*NB,1);  TODO!
    POS = zeros(3*NB,1);
    QUAT = zeros(4*NB,1); 

    % Gather vertex data to pass to Bullet 
    for b=1:sim.num_bodies
        body = sim.bodies(b);  
        if b > 1
            VERT_offset(b) = VERT_offset(b-1) + body.num_verts; 
        else
            VERT_offset(b) = body.num_verts;
        end

        bID(b) = body.bodyID; % Should be equal to b

        % Verts
        for v=1:body.num_verts
            VERTS(end+1) = body.verts_local(v,1); 
            VERTS(end+1) = body.verts_local(v,2);  
            VERTS(end+1) = body.verts_local(v,3); 
        end

        % Position
        POS(3*(b-1)+1) = body.u(1);
        POS(3*(b-1)+2) = body.u(2);
        POS(3*(b-1)+3) = body.u(3);

        % Quaternion
        QUAT(4*(b-1)+1) = body.quat(2); 
        QUAT(4*(b-1)+2) = body.quat(3);
        QUAT(4*(b-1)+3) = body.quat(4);
        QUAT(4*(b-1)+4) = body.quat(1);   % Note: Bullet puts the scalar last!
    end

    % Gather contacts from Bullet
    C = BULLET( VERTS, VERT_offset, POS, QUAT, bID, NB, length(VERTS) );
    contacts = [];
    
    nc = length(C)/10; 
    for i=1:nc
       b=10*(i-1);                      % offset
       B1id = C(b+2);                   % Body IDs
       B2id = C(b+3);
       if sim.P{B1id}.static && sim.P{B2id}.static
           continue;
       end
       n = -[C(b+4); C(b+5); C(b+6)];   % Normal
       p2 = [C(b+7); C(b+8); C(b+9)] -sim.P{B2id}.u;   % Point on body2
       psi = C(b+10);                   % Psi_n
       p1 = p2 +sim.P{B2id}.u - psi*n -sim.P{B1id}.u;  % Point on body1

       contacts = [contacts Contact(B1id,B2id,p1,n,psi)];
    end

    sim.contacs = contacts; 

end










