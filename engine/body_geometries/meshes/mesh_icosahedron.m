%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_icosahedron.m 
%
% Generates an icosahedron mesh object

function icosahedron = mesh_icosahedron()
  icosahedron = mesh_read_poly_file('icosahedron.poly'); 
  icosahedron.J = eye(3) * (1/20)*(3+sqrt(5));
  %icosahedron.Jinv = inv(icosahedron.J);
end

