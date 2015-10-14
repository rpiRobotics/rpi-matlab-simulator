

% Given a set of contacts C, determines a primary contact and returns the
% index i of that contact.  
function i = peg_heuristic( C )

    % Right now, return the largest non-negative contact
    N = length([C.psi_n]); 
    if N <=1
        i = N;
        return;
    end
    
    tolerance = -10^-4;
    c = C(1);
    i = 1;
    for j=2:N
       if C(j).psi_n >= tolerance && C(j).psi_n < c.psi_n
          c = C(j);  
          i = j; 
       end
    end

end

