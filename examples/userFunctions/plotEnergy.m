%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = plotEnergy( sim )

    body = sim.bodies(2);   % The body for which we'll plot the energy
    
    % Grab some body properties
    m = body.mass; 
    M = body_massInertiaMatrix( body );
    J = M(4:6,4:6);
    v = body.nu(1:3);   % Linear velocity
    w = body.nu(4:6);   % Rotational velocity

    % Energies 
    PE = m * norm(sim.gravityVector) * body.u(3); 
    KE = 0.5 * ( m * (v'*v) + w'*J*w );
    TE = PE+KE;
    
    if sim.step == 1
        % On the first time step, the plot doesn't exist yet, so we'll create it.
        figure(); 
        sim.userData.PE = plot(sim.time,PE);  
        hold on;      % Always remember hold on!
        sim.userData.KE = plot(sim.time,KE,'r'); 
        sim.userData.TE = plot(sim.time,TE,'g'); 
        xlabel('Time (s)'); 
        ylabel('Energy');
        legend('Potential energy','Kinetic energy', 'Total energy',2);
    else
        % After the first time step the plots exists, so we'll just update
        % their X and Y data.  
        set(sim.userData.PE,'xdata',[get(sim.userData.PE,'xdata') sim.time]);
        set(sim.userData.PE,'ydata',[get(sim.userData.PE,'ydata') PE]); 
        
        set(sim.userData.KE,'xdata',[get(sim.userData.KE,'xdata') sim.time]);
        set(sim.userData.KE,'ydata',[get(sim.userData.KE,'ydata') KE]); 
        
        set(sim.userData.TE,'xdata',[get(sim.userData.TE,'xdata') sim.time]);
        set(sim.userData.TE,'ydata',[get(sim.userData.TE,'ydata') TE]); 
    end


end

