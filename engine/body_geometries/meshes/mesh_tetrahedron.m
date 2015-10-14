%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_tetrahedron.m 
%
% Generates an tetrahedron mesh object

function tetrahedron = mesh_tetrahedron()
  tetrahedron = mesh_read_poly_file('tetrahedron.poly'); 
  tetrahedron.J = eye(3)*(1/20); 
  %tetrahedron.Jinv = inv(tetrahedron.J);
end

