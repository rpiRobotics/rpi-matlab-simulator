

% Returns a cross contact constraint given contact INDICIES C1 and C2.  

function c = constraint_cross_contact( C1, C2 )

    %c.constraint_type = 'crosscontact';
    c.C1 = C1;
    c.C2 = C2;

end

