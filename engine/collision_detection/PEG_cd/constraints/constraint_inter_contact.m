

% Returns a unilateral constraint given contact INDECIES C1 and C2.  

function ic = constraint_inter_contact( C1, C2 )

    %ic.constraint_type = 'intercontact';
    ic.C1 = C1;
    ic.C2 = C2;

end

