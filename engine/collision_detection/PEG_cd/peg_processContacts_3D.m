

% After collision detection finds all v-f and e-e contacts, this will
% process all contacts into U and I-constraints.  
function sim = peg_processContacts_3D( sim )

    C = sim.contacts; 
    num_vf = sim.userData.numVertContacts; 
    num_ee = length(C) - num_vf;  
    
    disp(['num_vf = ' num2str(num_vf) ', num_ee = ' num2str(num_ee)]);
    
    % Detect cases of vf, ee, ve, vv
    
    %% Vertex-face
    % Find all cases of vf where v does not have other contacts
    vf
    for v=1:num_vf
        
    end
    
    
%    Cvf = 

end

