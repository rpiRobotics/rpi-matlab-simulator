

% Returns a unilateral constraint given contact INDEX c.  

function u = constraint_unilateral( c )

    %u.constraint_type = 'unilateral';
    u.C = c;

end

