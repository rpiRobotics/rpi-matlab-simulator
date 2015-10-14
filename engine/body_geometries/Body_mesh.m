%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% By default, returns an empty mesh struct.  
% INPUTS 
%       [none] - returns an empty mesh
%       V      - Nx3 matrix of vertex data, for N vertices
%       F      - Mx3 matrix of triangle face data, for M faces

function body = Body_mesh( varargin )

    body = Body();
    body.type = 'mesh';  

    % Vertex and face data 
    if nargin > 0
        body.verts_local = varargin{1};                             % First argument must be Vertex data
        body.faces = [varargin{2} zeros(size(varargin{2},1),1)];    % Second argument must be Face data
        body.num_verts = size(body.verts_local,1);
        body.num_faces = size(body.faces,1); 

        % Generate edges and calculate tvecs
        body.edges = zeros((3/2)*body.num_faces,5);  % edge e = [vt vh f1 f2]
        eid = 1;
        for f=1:body.num_faces
            fv1 = body.faces(f,1);
            fv2 = body.faces(f,2);
            fv3 = body.faces(f,3); 

            % First edge
            e1 = find(ismember(body.edges(:,1:2),[fv2 fv1],'rows'),1);
            if any( e1 )    
                body.edges(e1,4) = f;       % face_2
            else            
                body.edges(eid,1) = fv1;    % v_tail 
                body.edges(eid,2) = fv2;    % v_head
                body.edges(eid,3) = f;      % face_1 
                eid = eid+1;
            end

            % Second edge
            e2 = find(ismember(body.edges(:,1:2),[fv3 fv2],'rows'),1);
            if any( e2 )
                body.edges(e2,4) = f;      
            else
                body.edges(eid,1) = fv2;    % Assign f1
                body.edges(eid,2) = fv3; 
                body.edges(eid,3) = f;
                eid = eid+1;
            end

            % Third edge
            e3 = find(ismember(body.edges(:,1:2),[fv1 fv3],'rows'),1);
            if any( e3 )
                body.edges(e3,4) = f;       % Assign f2
            else
                body.edges(eid,1) = fv3;    % Assign f1
                body.edges(eid,2) = fv1; 
                body.edges(eid,3) = f;
                eid = eid+1;
            end
        end
        body.num_edges = size(body.edges,1);
        body.tvecs = zeros(body.num_edges,6);
        

        % Generate half-edge datastructure
%        body.edges = zeros(3*body.num_faces,5);
%        eid = 1;
%        for f=1:body.num_faces 
%            fv1 = body.faces(f,1);
%            fv2 = body.faces(f,2);
%            fv3 = body.faces(f,3); 
%
%            % First edge
%            body.edges(eid,1) = fv1;    % v_tail
%            body.edges(eid,2) = fv2;    % v_head
%            body.edges(eid,3) = eid+1;  % e_next
%            body.edges(eid,5) = f;      % face 
%            eid = eid+1;
%
%            % Second edge
%            body.edges(eid,1) = fv2;    % v_tail
%            body.edges(eid,2) = fv3;    % v_head
%            body.edges(eid,3) = eid+1;  % e_next
%            body.edges(eid,5) = f;      % face 
%            eid = eid+1;
%
%            % Third edge
%            body.edges(eid,1) = fv3;    % v_tail
%            body.edges(eid,2) = fv1;    % v_head
%            body.edges(eid,3) = eid-2;  % e_next
%            body.edges(eid,5) = f;      % face 
%            eid = eid+1;
%        end
%        edgeCopy = body.edges(:,1:2);
%        for eid = 1:3*body.num_faces
%            if body.edges(eid,4) != 0, continue; end
%
%            ev1 = body.edges(eid,1);
%            ev2 = body.edges(eid,2); 
%            e_opp = find(ismember(edgeCopy,[ev2 ev1],'rows'),1);   % Find opposite edge
%            body.edges(eid,4) = e_opp;
%            body.edges(e_opp,4) = eid;    
%        end

    end % end nargin
    
end






