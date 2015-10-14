%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Given a body B, draws it and returns the updated body with its graphics
% handle. 
function B = body_draw_init( B )

    %% MESH
    if strcmp( B.type, 'mesh' )
        % Init draw
        h = patch('Vertices',B.verts_local,'Faces',B.faces(:,1:3)); 
        B.Xdata = get(h,'xdata');
        B.Ydata = get(h,'ydata');
        B.Zdata = get(h,'zdata'); 
        
    %% SPHERE
    elseif strcmp( B.type, 'sphere' )
        [B.Xdata, B.Ydata, B.Zdata] = sphere(B.num_sphere_verts); 
        B.Xdata = B.Xdata .* B.radius;
        B.Ydata = B.Ydata .* B.radius;
        B.Zdata = B.Zdata .* B.radius;
        h = surf(B.Xdata, B.Ydata, B.Zdata);
        B.num_faces = B.num_sphere_verts + 1; 
        
    %% PLANE
    elseif strcmp( B.type, 'plane' )
       scl = 3;                       % An arbitrary size
                
       t1 = arbitraryTangent(B.plane_normal);  % An arbitrary tangent
       t2 = rot(B.plane_normal,pi/2)*t1;
       t3 = rot(B.plane_normal,pi/2)*t2;
       t4 = rot(B.plane_normal,pi/2)*t3; 

       B.Xdata = scl*[t1(1) t2(1) t3(1) t4(1)];
       B.Ydata = scl*[t1(2) t2(2) t3(2) t4(2)];
       B.Zdata = scl*[t1(3) t2(3) t3(3) t4(3)];
       h = patch('XData',B.Xdata,'YData',B.Ydata,'ZData',B.Zdata); 
       B.num_faces = 4; 
    else
        error('Unknown body type');
    end

    
    set(h,'facecolor',B.color); 
    set(h,'facealpha',B.facealpha);
    set(h,'edgealpha',B.edgealpha); 
    if ~B.visible, set(h,'visible','off'); end 
    B.graphicsHandle = h; 
    
    body_draw( B ); % Draw again at world coordinates

end

