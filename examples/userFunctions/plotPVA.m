%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = plotPVA( sim )

    body = sim.bodies(2);   % The body for which we'll plot P, V, A in the z direction.
    
    P = body.u(3);              % Verticle position
    V = body.nu(3);             % Verticle velocity
    A = body.Fext(3)/body.mass; % Verticle acceleration
    
    if sim.step == 1
        % On the first time step, the plot doesn't exist yet, so we'll create it.
        figure(); 
        sim.userData.P = plot(sim.time, P, 'r');   hold on;
        sim.userData.V = plot(sim.time, V, 'g'); 
        sim.userData.A = plot(sim.time, A, 'b'); 
        xlabel('Time (s)'); 
        legend('Position','Velocity', 'Acceleration',2);
    else
        % After the first time step the plots exists, so we'll just update
        % their X and Y data.  
        set(sim.userData.P,'xdata',[get(sim.userData.P,'xdata') sim.time]);
        set(sim.userData.P,'ydata',[get(sim.userData.P,'ydata') P]); 
        
        set(sim.userData.V,'xdata',[get(sim.userData.V,'xdata') sim.time]);
        set(sim.userData.V,'ydata',[get(sim.userData.V,'ydata') V]); 
        
        set(sim.userData.A,'xdata',[get(sim.userData.A,'xdata') sim.time]);
        set(sim.userData.A,'ydata',[get(sim.userData.A,'ydata') A]); 
    end


end

