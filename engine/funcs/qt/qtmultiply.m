
% Return the *UNIT* quaternion multiplication of q and r.
function qout = qtmultiply(q,r)
    if size(q,1)~=4 || size(r,1)~=4, error('q and r must be 4-by-1'); end
    
    qout = [ q(1)*r(1) - q(2)*r(2) - q(3)*r(3) - q(4)*r(4)
             q(1)*r(2) + q(2)*r(1) + q(3)*r(4) - q(4)*r(3) 
             q(1)*r(3) - q(2)*r(4) + q(3)*r(1) + q(4)*r(2)
             q(1)*r(4) + q(2)*r(3) - q(3)*r(2) + q(4)*r(1) ];
         
%     qout = [ q(1)*r(1) - q(2)*r(2) - q(3)*r(3) - q(4)*r(4)
%              q(2)*r(1) + q(1)*r(2) - q(4)*r(3) + q(3)*r(4)
%              q(3)*r(1) + q(4)*r(2) + q(1)*r(3) - q(2)*r(4)
%              q(4)*r(1) - q(3)*r(2) + q(2)*r(3) + q(1)*r(4) ]; 
        
    qout = qout / norm(qout); 
end       
