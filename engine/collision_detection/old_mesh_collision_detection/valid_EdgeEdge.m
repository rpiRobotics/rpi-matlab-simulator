%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% valid_EdgeEdge.m 
%
% Inputs
%   B1: first body (body with edge e1)
%   B2: 2nd body (body with edge e2)
%   e1x: Body 1's edge INDEX (not edge object) 
%   e2x: Body 2's edge index
%   ep1: World coord of contact on e1
%   ep2: World coord of contact on e2
%
% Output
%   isValid: true is 

function isValid = valid_EdgeEdge( B1, B2, e1x, e2x, ep1, ep2 )
    isValid = true;
    
    % First attempt: if all face normals are facing the same way, then the
    % edge is on the wrong side of the body (I think).  EDIT: Ok, this was just wrong
%     na1 = B1.faces(B1.edges(e1x).faces(1)).normal;
%     na2 = B1.faces(B1.edges(e1x).faces(2)).normal;
%     nb1 = B2.faces(B2.edges(e2x).faces(1)).normal;
%     nb2 = B2.faces(B2.edges(e2x).faces(2)).normal;
%     if dot3(na1,nb1)>0 && dot3(na1,nb2)>0 && dot3(na2,nb1)>0 && dot3(na2,nb2)>0
%        isValid = false;
%     end
    % Second attempt: if the edge normals (average of face normals) of both
    % bodies are facing the same way, then the edges aren't touching.
    na1 = B1.faces(B1.edges(e1x).faces(1)).normal;
    na2 = B1.faces(B1.edges(e1x).faces(2)).normal;
    nb1 = B2.faces(B2.edges(e2x).faces(1)).normal;
    nb2 = B2.faces(B2.edges(e2x).faces(2)).normal;
    if dot(na1+na2,nb1+nb2) > 10^-5
       isValid = false;  
    end

end

