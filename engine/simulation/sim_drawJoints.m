%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = sim_drawJoints( sim )

    for j = 1:length(sim.joints)
        Jnt = sim.joints(j); 
        
        p1 = Jnt.P1; 
        x1 = Jnt.X1;
        y1 = Jnt.Y1;
        z1 = Jnt.Z1; 
        p2 = Jnt.P2;
        x2 = Jnt.X2;
        y2 = Jnt.Y2;
        z2 = Jnt.Z2; 

        set(Jnt.P1Handle,'XData',p1(1),'YData',p1(2),'ZData',p1(3)); 
        set(Jnt.P1_X_AxisHandle,'XData',[p1(1) x1(1)],'YData',[p1(2) x1(2)],'ZData',[p1(3) x1(3)]);
        set(Jnt.P1_Y_AxisHandle,'XData',[p1(1) y1(1)],'YData',[p1(2) y1(2)],'ZData',[p1(3) y1(3)]);
        set(Jnt.P1_Z_AxisHandle,'XData',[p1(1) z1(1)],'YData',[p1(2) z1(2)],'ZData',[p1(3) z1(3)]);
        %set(Jnt.P1_Z_pointHandle,'XData',z1(1),'YData',z1(2),'ZData',z1(3)); 
        set(Jnt.P2Handle,'XData',p2(1),'YData',p2(2),'ZData',p2(3)); 
        set(Jnt.P2_X_AxisHandle,'XData',[p2(1) x2(1)],'YData',[p2(2) x2(2)],'ZData',[p2(3) x2(3)]);
        set(Jnt.P2_Y_AxisHandle,'XData',[p2(1) y2(1)],'YData',[p2(2) y2(2)],'ZData',[p2(3) y2(3)]);
        set(Jnt.P2_Z_AxisHandle,'XData',[p2(1) z2(1)],'YData',[p2(2) z2(2)],'ZData',[p2(3) z2(3)]);
        %set(Jnt.P2_Z_pointHandle,'XData',z2(1),'YData',z2(2),'ZData',z2(3)); 
        set(Jnt.labelHandle,'Position',p1+[0;0;0.5]); 
    end

end

