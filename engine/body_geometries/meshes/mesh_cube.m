%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_cube.m 
%
% Generates a cube mesh object

% TODO:   
%   - Take more arguments, e.g. mass. 

function cube = mesh_cube()
    cube = mesh_read_poly_file('cube.poly'); 
    cube.J = eye(3)*(1/6); 
    %cube.Jinv = inv(cube.J); 
end

