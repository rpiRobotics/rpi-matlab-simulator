
% Rotate a vector vin by quaternion q.  
% Also works on a matrix vin if it is 3-by-N
% Assumes the quaternion is of unit length. 
function vout = qtrotate(q,vin) 
  if(size(q,1) ~= 4), error('q must be 4-by-1'); end;
  if(size(vin,1) ~= 3), error('vin must be 3-by-N'); end;
  
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
  q4 = q(4);

  R = [ 1-2*(q3^2 + q4^2)   2*(q2*q3 - q4*q1)   2*(q2*q4 + q3*q1)
        2*(q2*q3 + q4*q1)   1-2*(q2^2 + q4^2)   2*(q3*q4 - q2*q1)
        2*(q2*q4 - q3*q1)   2*(q3*q4 + q2*q1)   1-2*(q2^2 + q3^2) ];
    
  for i = size(vin,2):-1:1
    vout(:,i) = (R*vin(:,i));
  end
  
end


