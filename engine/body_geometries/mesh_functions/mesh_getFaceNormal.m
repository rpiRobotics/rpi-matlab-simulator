

% Given a body B and face index f, returns the face normal of f.
% By default, this returns the normal in the world frame, however
% if varargin{3} is true, it returns the normal in the local frame.

function n = mesh_faceNormal( B, f, varargin )

    if nargin > 2 && varargin{3}
        v1 = B.verts_local(B.faces(f,1),:);
        v2 = B.verts_local(B.faces(f,2),:);
        v3 = B.verts_local(B.faces(f,3),:);
    else
        v1 = B.verts_world(B.faces(f,1),:);
        v2 = B.verts_world(B.faces(f,2),:);
        v3 = B.verts_world(B.faces(f,3),:);
    end

    n = cross3( v3-v2 , v1-v2  );
    n = n / norm(n); 

end


