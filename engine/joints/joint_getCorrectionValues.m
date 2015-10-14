%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function [Gb C Cdot] = joint_getCorrectionValues( sim, j )

   Jnt = sim.joints(j); 
   body1 = sim.bodies(Jnt.body1id); 
   body2 = sim.bodies(Jnt.body2id); 
   
   p1 = Jnt.P1;             p2 = Jnt.P2;
   x1 = Jnt.X1-p1;          x2 = Jnt.X2-p2;
   y1 = Jnt.Y1-p1;          y2 = Jnt.Y2-p2;
   z1 = Jnt.Z1-p1;          z2 = Jnt.Z2-p2; 
   r1 = p1-body1.u;         r2 = p2-body2.u; 
   r1z = Jnt.Z1-p1;         r2z = Jnt.Z2-p2; 

   Perr = p1-p2;            % Error of the joint origins
   Zerr = Jnt.Z1-Jnt.Z2;    % Error of the Z points
   Xerr = Jnt.X1-Jnt.X2;    % Error of the X points

   b1 = body1;          b2 = body2; 
   jntType = Jnt.jntCode;  

   switch jntType
       case 1         % Fixed
            if b1.static, G1 = []; else
                G1 = [       x1             y1            z1            x1             y1            z1
                       cross3(r1,x1) cross3(r1,y1) cross3(r1,z1) cross3(r1z,x1) cross3(r1z,y1) cross3(r1z,z1) ]; 
            end
            if b2.static, G2 = []; else
                G2 = [       -x2            -y2            -z2            -x2             -y2             -z2
                       cross3(r2,-x2) cross3(r2,-y2) cross3(r2,-z2) cross3(r2z,-x2) cross3(r2z,-y2) cross3(r2z,-z2) ]; 
            end
            Gb = [G1; G2]; 
            C = [ dot(Perr, x1)  % Constraint error
                  dot(Perr, y1)
                  dot(Perr, z1)
                  dot(Zerr, x1)
                  dot(Zerr, y1)
                  dot(Zerr, z1) ];  

       case 2      % Revolute
            if ~b1.dynamic, G1 = []; else
                G1 = [       x1             y1            z1            x1             y1    
                       cross3(r1,x1) cross3(r1,y1) cross3(r1,z1) cross3(r1z,x1) cross3(r1z,y1) ]; 
            end
            if ~b2.dynamic, G2 = []; else
                G2 = [       -x2            -y2            -z2            -x2             -y2 
                       cross3(r2,-x2) cross3(r2,-y2) cross3(r2,-z2) cross3(r2z,-x2) cross3(r2z,-y2) ]; 
            end
            Gb = [G1; G2]; 
            C = [ dot(Perr, x1)  % Constraint error
                  dot(Perr, y1)
                  dot(Perr, z1)
                  dot(Zerr, x1)
                  dot(Zerr, y1) ];  

       case 3      % Prismatic
            if b1.static, G1 = []; else
                r1x = Jnt.X1-p1;  % Prismatic is a special case where we constrain the Joints' X axes in the y direction
                G1 = [       x1             y1           x1             y1            y1
                       cross3(r1,x1) cross3(r1,y1) cross3(r1z,x1) cross3(r1z,y1) cross3(r1x,y1) ]; 
            end
            if b2.static, G2 = []; else
                r2x = Jnt.X2-p2; 
                G2 = [       -x2            -y2            -x2             -y2             -y2
                       cross3(r2,-x2) cross3(r2,-y2) cross3(r2z,-x2) cross3(r2z,-y2) cross3(r2x,-y2) ]; 
            end
            Gb = [G1; G2]; 
            C = [ dot(Perr, x1)  % Constraint error
                  dot(Perr, y1)
                  dot(Zerr, x1)
                  dot(Zerr, y1)
                  dot(Xerr, y1) ];  

       case 4      % Cylindrical
           if b1.static, G1 = []; else
                G1 = [       x1             y1           x1             y1      
                       cross3(r1,x1) cross3(r1,y1) cross3(r1z,x1) cross3(r1z,y1) ]; 
            end
            if b2.static, G2 = []; else
                G2 = [       -x2            -y2            -x2             -y2      
                       cross3(r2,-x2) cross3(r2,-y2) cross3(r2z,-x2) cross3(r2z,-y2) ]; 
            end
            Gb = [G1; G2]; 
            C = [ dot(Perr, x1)  
                  dot(Perr, y1)
                  dot(Zerr, x1)
                  dot(Zerr, y1) ];  

       case 5      % Spherical
           if b1.static, G1 = []; else
            G1 = [       x1             y1            z1             
                   cross3(r1,x1) cross3(r1,y1) cross3(r1,z1) ]; 
            end
            if b2.static, G2 = []; else
            G2 = [       -x2            -y2            -z2     
                   cross3(r2,-x2) cross3(r2,-y2) cross3(r2,-z2) ]; 
            end
            Gb = [G1; G2]; 
            C = [ dot(Perr, x1)  
                  dot(Perr, y1)
                  dot(Perr, z1) ];  
   end

   % Determine Cdot
   if ~body1.dynamic 
       nu = body2.nu;
   elseif ~body2.dynamic 
       nu = body1.nu;
   else
       nu = [body1.nu; body2.nu];
   end
   Cdot = Gb' * nu; 


end

