
% Draws a body, returns the graphics handle

function body_draw( B )

    % Update graphics
    % Transform data to world frame coordinates
    X = zeros(size(B.Xdata));
    Y = zeros(size(B.Ydata));
    Z = zeros(size(B.Zdata));
    for i=1:B.num_faces
       D = qtrotate(B.quat, [B.Xdata(:,i) B.Ydata(:,i) B.Zdata(:,i)]')';
       X(:,i) = D(:,1) + B.u(1);
       Y(:,i) = D(:,2) + B.u(2);
       Z(:,i) = D(:,3) + B.u(3);
    end
    
    % Here, we exploit MATLAB's non-case-sensitivity to match Octave's
    % case-sensitivity for 'xdata' etc. in the graphics data.  
    set(B.graphicsHandle,'xdata',X);
    set(B.graphicsHandle,'ydata',Y);
    set(B.graphicsHandle,'zdata',Z);
    set(B.graphicsHandle,'facecolor',B.color); 

end



