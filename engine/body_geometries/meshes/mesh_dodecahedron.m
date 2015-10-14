%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_dodecahedron.m 
%
% Generates a dodecahedron mesh object

function dodecahedron = mesh_dodecahedron()
  dodecahedron = mesh_read_poly_file('dodecahedron.poly'); 
  dodecahedron.J = eye(3) * (1/300)*(95+39*sqrt(5)); 
  %dodecahedron.Jinv = inv(dodecahedron.J);
end

