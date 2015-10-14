
% Returns a 3x3 rotation matrix representing 
% rotation of theta radians about column vector k. 
function R=rot(k,theta)
  k=k/norm(k);
  if size(k,2) > 1, k = k'; end
  %R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
  % From Peter Corke, replace "hat(k)*hat(k)" with "(k*k'-eye(3))". 
  % Makes it about ~1.8 times faster.  
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*(k*k'-eye(3));
end
