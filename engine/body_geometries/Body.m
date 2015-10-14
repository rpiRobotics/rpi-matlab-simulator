
% Creates a body struct with default body properties

function B = Body()

    % The body struct
    B = struct;
    
    B.name = 'body';
    B.type = 'generic';

    B.bodyID = 0;           % Body's position in the simulation's vector of bodies
    B.bodyContactID = 0;    % Body's id for current time-step contact set
    B.bodyDynamicID = 0;    % TODO: duplicate of bodyContactID? 
    
    % Physical properties
    B.dynamic = true;       % Dynamic vs. static body 
    B.active = false;       % An active body is one that is in dynamic interaction with 
                            %   another body through contact or some other constraint.  
    B.mass = 1;             % Mass
    B.u = zeros(3,1);       % Position of center of mass
    B.quat = [1;0;0;0];     % Quaternion rotation
    B.J = eye(3);           % Inertia tensor
    %B.v = zeros(3,1);       % Linear velocity
    %B.w = zeros(3,1);       % Angular velocity
    B.nu = zeros(6,1);      % Generalized velocity [v; w]

    B.Fext = zeros(6,1);    % External force
    B.Aext = zeros(6,1);    % External acceleration
    
    B.mu = 0.5;             % Friction factor
    
    % Joint-related properties
    B.numJoints = 0; 
    
    % Graphics properties 
    B.visible = true; 
    B.graphicsHandle = [];  % Graphics object for plotting the body
    B.Xdata = [];
    B.Ydata = [];
    B.Zdata = [];
    B.color = [0 0 1];      % Color of body
    B.facealpha = 0.7;
    B.edgealpha = 1; 
    B.bboxHandle = [];      % Graphics object for plotting bounding box

    % Collision detection properties
    B.AABB_min = zeros(1,3); % Bounding box
    B.AABB_max = zeros(1,3); 

    B.collides = true;         % If false, this body won't collide with any other bodies
    B.doesNotCollideWith = []; % A list of bodyIDs of bodies not to do collision detection with
    
    
    %% Different bodies have different attributes, but for simplicity 
    %  we want all bodies to be represented by this struct.  Therefore we
    %  place all body attributes here, and let bodies pick and choose.
    
    %% Sphere attributes
    B.radius = 1;
    B.num_sphere_verts = 7;  % Decrease to improve graphics performance 
                             % (does not affect simulation result)
    
    %% Cylinder attributes
    B.height = 1; 
                             
    %% Plane attributes
    B.plane_normal = [0 0 1];
    
    %% Mesh attributes
    B.num_verts = 0;
    B.num_edges = 0;
    B.num_faces = 0; 

    B.verts_local = [];  % A Nx3 vector of N vertices relative to the COM of the body
    B.verts_world = [];  % A Nx3 vector of N vertices in the world space
    B.edges = [];        % A Ex [v1 v2 Enext Eopp face]   TODO: remove implicit columns Enext and face
                            % A Ex5 vector [v1 v2 f1 f2 a] where v1:first
                            % vert, v2: second vert, f1: first face, f2:
                            % second face, a: angle between faces
    B.tvecs = [];        % A Ex6 vector of t vectors (always in local coords), perp to edge and planar to adjacent faces
    B.faces = [];        % A Mx4 vector of M faces where each row contains: [v1 v2 v3 e1]
    B.face_norms = [];   % A Mx3 vector of M face norms where each row contains [n1 n2 n3] 
    
    
end

