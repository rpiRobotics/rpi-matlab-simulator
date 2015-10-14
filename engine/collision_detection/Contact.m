%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% Contact.m 
%
% Represents a contact between two objects

function c = Contact( body1_id, body2_id, p1, normal, psi_n )
    c.body1_id = body1_id;  % bodyID of first body in contact
    c.body2_id = body2_id;  % bodyID of second body in contact
    c.p1 = p1;              % The point of contact (in world space) on body_1 (must be a row vector)
    c.normal = normal;      % Normal direction (from body_1 to body_2) (must be a row vector)
    c.psi_n = psi_n;        % Gap distance (negative -> penetration depth) 
end
 

