%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% This function initializes all of the graphics associated with a
% simulation. 

function sim = sim_draw_init( sim )

    figure();  hold on; 
    
    %% Init body graphics
    for b=1:length(sim.bodies)
       sim.bodies(b) = body_draw_init(sim.bodies(b));  
    end
    
    %% Init bounding box graphics
    if sim.drawBoundingBoxes 
        Vbb = zeros(8,3); 
        for b=1:sim.num_bodies
            M = sim.bodies(b).AABB_max;
            m = sim.bodies(b).AABB_min; 
            Vbb(1,:) = [m(1) m(2) m(3)];  % Verts of BBox
            Vbb(2,:) = [m(1) M(2) m(3)];
            Vbb(3,:) = [M(1) M(2) m(3)];
            Vbb(4,:) = [M(1) m(2) m(3)];
            Vbb(5,:) = [m(1) m(2) M(3)];
            Vbb(6,:) = [m(1) M(2) M(3)];
            Vbb(7,:) = [M(1) M(2) M(3)];
            Vbb(8,:) = [M(1) m(2) M(3)];
            Ebb = [ Vbb(1,:); Vbb(2,:)
                    Vbb(3,:); Vbb(4,:)
                    Vbb(1,:); Vbb(5,:)
                    Vbb(6,:); Vbb(7,:)
                    Vbb(8,:); Vbb(5,:)
                    Vbb(6,:); Vbb(2,:)
                    Vbb(3,:); Vbb(7,:)
                    Vbb(8,:); Vbb(4,:) ];
            sim.bodies(b).bboxHandle = plot3(Ebb(:,1),Ebb(:,2),Ebb(:,3),'b');
        end
    end
    
    %% Init joint graphics
    if sim.drawJoints
        for j=1:length(sim.joints)
           Jnt = sim.joints(j); 
           p1 = Jnt.P1; 
           x1 = Jnt.X1;
           y1 = Jnt.Y1;
           z1 = Jnt.Z1; 
           p2 = Jnt.P2;
           x2 = Jnt.X2;
           y2 = Jnt.Y2;
           z2 = Jnt.Z2; 
           
          Jnt.drawn = true; 
          Jnt.P1Handle = plot3(p1(1),p1(2),p1(3),'r*');
          Jnt.P1_X_AxisHandle = plot3([p1(1) x1(1)],[p1(2) x1(2)],[p1(3) x1(3)],'r');
          Jnt.P1_Y_AxisHandle = plot3([p1(1) y1(1)],[p1(2) y1(2)],[p1(3) y1(3)],'g');
          Jnt.P1_Z_AxisHandle = plot3([p1(1) z1(1)],[p1(2) z1(2)],[p1(3) z1(3)],'b');
          %Jnt.P1_Z_pointHandle = plot3(z1(1),z1(2),z1(3),'bo');

          Jnt.P2Handle = plot3(p2(1),p2(2),p2(3),'r*');
          Jnt.P2_X_AxisHandle = plot3([p2(1) x2(1)],[p2(2) x2(2)],[p2(3) x2(3)],'r');
          Jnt.P2_Y_AxisHandle = plot3([p2(1) y2(1)],[p2(2) y2(2)],[p2(3) y2(3)],'g');
          Jnt.P2_Z_AxisHandle = plot3([p2(1) z2(1)],[p2(2) z2(2)],[p2(3) z2(3)],'b');
          %Jnt.P2_Z_pointHandle = plot3(z2(1),z2(2),z2(3),'bo');

          Jnt.labelHandle = text(p1(1),p1(2),p1(3)+0.5,['Joint ' num2str(Jnt.jointID)]);

          sim.joints(j) = Jnt; 
        end
    end
    
    %% Set default plot properties
    view(3); 
    grid on; 
    axis equal; 
    xlabel('X'); ylabel('Y'); zlabel('Z'); 
    
    if ~exist('OCTAVE_VERSION','var'), rotate3d; end

end

