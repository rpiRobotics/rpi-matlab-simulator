

% INPUT:
%   A       - A body
%   B       - A body
%   va      - Index of vertex on A
%   fb      - Index of face on B
%   eps_vf  - Distance vertex must be within fb to be considered as potential contact
%
% OUTPUT: 
%   FEAS - A boolean, true if va has FEASIBILITY with fb


function FEAS = feasibility_vf( A, B, va, fb, eps_vf )

    epsilon_small = 0.0025;
    epsilon_theta = .4; %(.4rad = 22.9deg) 
    
    FEAS = true; 
    
    % Face Fb
    Fb = B.faces(fb,:);     
    nb = B.face_norms(fb,:);

    % Vertex Va
    Va = A.verts_world(va,:);
    
    %% Determine feasibiltiy
    psi = dot3( nb, Va - B.verts_world(Fb(1),:) );
    if abs(psi) > eps_vf
       FEAS = false;
       return; 
    end
    
    [D,Pt] = point_triangle_distance_3d( [B.verts_world(Fb(1),:) 
                                          B.verts_world(Fb(2),:)
                                          B.verts_world(Fb(3),:)] ,Va);
    
    % Do an epsilon check TODO: Is this redundant? 
    if D > eps_vf
       FEAS = false; 
       return;
    end
    
    %% TODO: What about case where va is on +psi of an adjacent face????
    % That is, near the edge eb, with small -psi on Fb, but in voronoi
    % region of the adjacent face to Fb across eb?
    vb1 = B.verts_world(Fb(1),:);
    vb2 = B.verts_world(Fb(2),:);
    vb3 = B.verts_world(Fb(3),:);
    [d1,p1] = point_segment_distance_3D(vb1, vb2, Va);  % Edge eb1
    [d2,p2] = point_segment_distance_3D(vb2, vb3, Va);  % Edge eb2
    [d3,p3] = point_segment_distance_3D(vb3, vb1, Va);  % Edge eb3
    
    if d1 < d2
       if d1 < d3
           D = d1; P = p1;
           eb = vb2-vb1;
           edgePoint = vb1;
       else
           D = d3; P = p3;
           eb = vb1-vb3;
           edgePoint = vb3;
       end
    else  
       if d2 < d3
           D = d2; P = p2;
           eb = vb3-vb2; 
           edgePoint = vb2;
       else
           D = d3; P = p3; 
           eb = vb1-vb3;
           edgePoint = vb3;
       end
    end
    
    % TODO: ! Va might be nearest a vertex, in which case outvec is wrong
    outvec = cross3(eb,nb);         % Vector pointint "out" of face at eb
    outvec = outvec/norm(outvec); 
    outDist = dot3(outvec,Va-edgePoint);  % Distance outside of face V region (negative value means inside V region)
    
    if outDist > eps_vf
       FEAS = false; 
       return;
    end
    
    % Point is "inside" face   (TODO: this doesn't work since outvec is wrong)
    %                           consider using point_triangle_information()
    if outDist < 0 
       if psi < -epsilon_small
            FEAS = false; 
       end
       return; 
    end

    % Va is outside face, check if it's above %theta from face
    if asin(psi/D) < -epsilon_theta
        FEAS = false;
    end
    
end













