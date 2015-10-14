%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function body = Body_sphere( mass, radius )

    body = Body();
    body.type = 'sphere'; 
    body.radius = radius; 
    body.J = (2*mass*radius^2 / 5) * eye(3);

end

