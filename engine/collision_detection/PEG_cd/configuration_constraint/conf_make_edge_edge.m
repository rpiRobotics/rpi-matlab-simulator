

% INPUT:
%   A   - Body A
%   B   - Body B
%   ea  - Edge index for edge on A
%   eb  - Edge index for edge on B
%   pa  - Point on ea nearest to eb
%   pb  - Point on eb nearest to ea

function C = conf_make_edge_edge( A,B,ea,eb,pa,pb ) 

    C = [];
    
    if applicability_ee(A,B,ea,eb) > -.4
        [orienation n] = orientation_ee( A, B, ea, eb );
        psi = dot3(n,pb-pa);
        C = Contact(A.bodyID,B.bodyID,pa,n,psi); 
        C.type = 'ee'; 
        C.f1id = ea;
        C.f2id = eb; 
    end

end

