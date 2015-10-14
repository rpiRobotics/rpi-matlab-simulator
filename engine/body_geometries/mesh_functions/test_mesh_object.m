%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%  
% Draws a mesh for inspection.  In particular, we are looking at face 
%   normals to confirm that faces were properly defined in .poly files.  
%

function test_mesh_object( m )
    
    % Draw m
    m.color = [.8 .1 .1];
    m = body_draw_init(m); 
    set(m.graphicsHandle,'FaceAlpha',0.6);
    axis equal;  
    hold on; 
    
    %num_verts = size(m.verts_local,1);
    num_faces = size(m.faces,1);

    % Draw face normals
    for f = 1:num_faces
        
        % Find center of face
        V = [ m.verts_local( m.faces(f,1), :);  m.verts_local( m.faces(f,2), :);  m.verts_local( m.faces(f,3), :)];
        u = mean(V,1); 
        
        % Plot face normal
        v1 = V(1,:);
        v2 = V(2,:);
        v3 = V(3,:); 
        n = cross(v3-v2,v1-v2);
        drawvec(u, u+n);
        un = u+n;
        plot3([u(1) un(1)],[u(2) un(2)],[u(3) un(3)]);
        
        % Check face (assumes the normal will be away from center)
        if dot( (u-[0 0 0]) , n) < 0
           disp(['Check face ' num2str(f)]); 
        end
    end

end

