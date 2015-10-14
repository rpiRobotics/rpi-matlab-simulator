

% INPUT:
%   A   - A body
%   B   - A body
%   ea  - Index of edge on A
%   eb  - Index of edge on B
%
% OUTPUT
%   APP - The applicability of ea on eb

function APP = applicability_ee( A, B, ea, eb )
    
    % Edges
    Ea = A.edges(ea,:);
    Eb = B.edges(eb,:);
    
    % Contact normal
    ea_vec = A.verts_world(Ea(1),:) - A.verts_world(Ea(2),:); 
    eb_vec = B.verts_world(Eb(1),:) - B.verts_world(Eb(2),:); 
    n = cross(ea_vec,eb_vec);
    if norm(n) == 0
       APP = -1;  % TODO: actually handle this case. 
       return; 
    end
    n = n/norm(n); 
    
    % T vectors
    Ta1 = A.tvecs(ea,1:3);
    Ta2 = A.tvecs(ea,4:6);
    Tb1 = B.tvecs(eb,1:3);
    Tb2 = B.tvecs(eb,4:6); 
    
    % Relaxation of classical applicability
    dTa1 = dot3(n,Ta1/norm(Ta1));
    dTa2 = dot3(n,Ta2/norm(Ta2));
    dTb1 = dot3(-n,Tb1/norm(Tb1));
    dTb2 = dot3(-n,Tb2/norm(Tb2));
    
    APP = max([min([dTa1 dTa2 dTb1 dTb2]) min(-[dTa1 dTa2 dTb1 dTb2])]); 
    
%     if max([dTa1 dTa2]) <= 10^-5
%        dTa1 = -dTa1;
%        dTa2 = -dTa2;
%     else
%        dTb1 = -dTb1;
%        dTb2 = -dTb2; 
%     end
%     
%     APP = min([dTa1 dTa2 dTb1 dTb2]); 
    
    
end








