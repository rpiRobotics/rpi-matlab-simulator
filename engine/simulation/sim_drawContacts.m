%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = sim_drawContacts( sim )

    % Clear previous collisions
    delete(sim.contactGraphics(:)); 
    sim.contactGraphics = []; 

    C = sim.contacts; 
    if ~isempty(C)
        
        %sim.contactGraphics = gobjects(3*length(C),1); gobjects is a MATLAB thing
        sim.contactGraphics = zeros(3*length(C),1); 
        % Draw all collisions
        for c=1:length(C)
            p1 = C(c).p1; 
            p2 = p1 + C(c).psi_n * C(c).normal; 

            if C(c).psi_n < 0
               sim.contactGraphics(3*c-2) = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r');
            else
               sim.contactGraphics(3*c-2) = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'g'); 
            end
            sim.contactGraphics(3*c-1) = plot3(p1(1),p1(2),p1(3),'go');
            sim.contactGraphics(3*c) = plot3(p2(1),p2(2),p2(3),'gx');
        end
    end
    
    
    % Update bounding boxes
    if sim.drawBoundingBoxes
        Vbb = zeros(3,8); 
        for b=1:sim.num_bodies
            if sim.bodies(b).dynamic 
                M = sim.bodies(b).AABB_max;
                m = sim.bodies(b).AABB_min; 

                Vbb(:,1) = [m(1); m(2); m(3)];  % Verts of BBox
                Vbb(:,2) = [m(1); M(2); m(3)];
                Vbb(:,3) = [M(1); M(2); m(3)];
                Vbb(:,4) = [M(1); m(2); m(3)];
                Vbb(:,5) = [m(1); m(2); M(3)];
                Vbb(:,6) = [m(1); M(2); M(3)];
                Vbb(:,7) = [M(1); M(2); M(3)];
                Vbb(:,8) = [M(1); m(2); M(3)];
                Ebb = [ Vbb(:,1) Vbb(:,2) Vbb(:,3) Vbb(:,4) Vbb(:,1) Vbb(:,5) ...
                        Vbb(:,6) Vbb(:,7) Vbb(:,8) Vbb(:,5) Vbb(:,6) Vbb(:,2) ...
                        Vbb(:,3) Vbb(:,7) Vbb(:,8) Vbb(:,4) ];

                set(sim.bodies(b).bboxHandle,'xdata',Ebb(1,:)); 
                set(sim.bodies(b).bboxHandle,'ydata',Ebb(2,:)); 
                set(sim.bodies(b).bboxHandle,'zdata',Ebb(3,:)); 
            end
            if sim.bodies(b).active
               set(sim.bodies(b).bboxHandle,'color',[1 0 0]);
            else
               set(sim.bodies(b).bboxHandle,'color',[0 0 1]);
            end
        end
    end
    
end


