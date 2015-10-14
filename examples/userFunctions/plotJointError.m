%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = plotJointError( sim )

    [C,~] = joint_constraintError(sim,1); 
    if sim.step == 1
       figure(); 
       sim.data.error = plot(sim.time,norm(C));
       xlabel('Time (s)'); 
       ylabel('norm(Joint Error)');
    else
        set(sim.data.error,'xdata',[get(sim.data.error,'xdata') sim.time]);
        set(sim.data.error,'ydata',[get(sim.data.error,'ydata') norm(C)]); 
    end


end


