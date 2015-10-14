%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Creates a Simulator struct containing simulation properties

function sim = Simulator(varargin)
    
    % The simulator struct
    sim = struct; 

    % Simulator properties
    sim.time = 0; 
    sim.step = 0;
    sim.MAX_STEP = 10^9; 
    sim.gravity = true; 
    sim.gravityVector = [0; 0; -9.81]; 
    sim.jointCorrection = false; 
    sim.record = false; 
    sim.record_directory = ''; 
    sim.record_fileID = -1; 
    sim.FRICTION = true; 
    sim.num_fricdirs = 7; 
    
    sim.userFunction = [];      % A function handle to a user function
    sim.userData = struct;      % A struct for storing user data
  
    % Body properties
    sim.num_bodies = 0;
    sim.bodies = []; 
    
    % Function handles
    sim.H_collision_detection = @collision_detection; 
    sim.H_dynamics = @LCPdynamics;
    sim.H_solver = @pathsolver; 

    % Use PATH's courtesy license if one isn't set already.
    if isempty(getenv('PATH_LICENSE_STRING'))
        setenv('PATH_LICENSE_STRING','2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0');
    end

    sim.dynamics = struct;
    sim.contacts = []; 
    sim.joints = []; 
    sim.num_jointConstraints = 0;
    sim.num_activeJointBodies = 0;
    sim.num_activeBodies = 0; 
    sim.num_dynamicBodies = 0;

    % Store solution error
    sim.solution_error = [];
    
    % Collision detection properties
    sim.epsilon = 0.05; 

    % Graphics properties
    sim.figure = 1;
    sim.draw = true; 
    sim.drawContacts = false; 
    sim.drawBoundingBoxes = false; 
    sim.drawJoints = false; 
    sim.contactGraphics = []; 

    %% Parse input arguments
    if nargin == 0
        sim.h = 0.01;   % Timestep size in seconds
    else
        sim.h = varargin{1}; 
    end
    
end



