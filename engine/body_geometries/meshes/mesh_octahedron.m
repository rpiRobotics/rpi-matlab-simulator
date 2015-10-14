%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_octahedron.m 
%
% Generates a octahedron mesh object

function octahedron = mesh_octahedron()
  octahedron = mesh_read_poly_file('octahedron.poly'); 
  octahedron.J = eye(3) * (1/10);
  %octahedron.Jinv = inv(octahedron.J);
end

