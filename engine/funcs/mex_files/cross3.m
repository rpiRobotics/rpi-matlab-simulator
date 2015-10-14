
% Calculates the cross product of 2 vectors A and B
% having a specific length of 3.  
% Note: This is a backup function in case cross3.c doesn't compile.  
function C = cross3(A,B)
  C = [ A(2)*B(3) - A(3)*B(2)
        A(3)*B(1) - A(1)*B(3)
        A(1)*B(2) - A(2)*B(1) ];
end
