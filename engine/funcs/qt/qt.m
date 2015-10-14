
% Returns a 4x1 vector representing a unit quaternion
% rotation of theta radians about vector k.
% Note: the scalar is the first element q(1). 
function q = qt( k, theta )
  k = k/norm(k);            % Normalize k
  if size(k,2) > 1
    q = [cos(theta/2); k'*sin(theta/2)];
  else
    q = [cos(theta/2); k*sin(theta/2)];
  end
  
