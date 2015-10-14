%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_record.m
%
% Writes simulation data to a directory for playback with sim_Reaplay(). 

function sim = sim_record( sim )
    if sim.step == 1
        sim.record_directory = ['sim_data_' datestr(now,'yy_mm_dd_HH_MM_SS')]; 
        mkdir(sim.record_directory);  % Create directory
        bodies = sim.bodies;                  % Record bodies
        save([sim.record_directory '/bodies.mat'],'bodies'); 
        % Open file for writing object position and orientation
        sim.record_fileID = fopen([sim.record_directory '/sim_log.txt'],'w');  
    end 
    fprintf(sim.record_fileID,'%f,',sim.time);
    for i=1:length(sim.bodies)
        fprintf(sim.record_fileID,'%f,%f,%f,%f,%f,%f,%f,',...
            sim.bodies(i).u(1),sim.bodies(i).u(2),sim.bodies(i).u(3),...
            sim.bodies(i).quat(1),sim.bodies(i).quat(2),sim.bodies(i).quat(3),sim.bodies(i).quat(4));
    end
    fprintf(sim.record_fileID,'\n'); 
end
