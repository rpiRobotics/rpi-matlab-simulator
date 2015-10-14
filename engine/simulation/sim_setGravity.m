%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%


function sim = sim_setGravity( sim )
    
    for b = 1:sim.num_bodies
        if sim.bodies(b).dynamic
            sim.bodies(b).Fext(1:3) = sim.bodies(b).Fext(1:3) + sim.gravityVector * sim.bodies(b).mass; 
        end
    end

end

