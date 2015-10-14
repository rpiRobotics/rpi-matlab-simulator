

function body = Body_plane( point, normal )

    body = Body();
    body.type = 'plane'; 
    body.color = [.6 .6 .6]; 
    body.dynamic = false;   % We require that all plane bodies are static
    
    if nargin > 0
       body.u = point;
       body.plane_normal = normal/norm(normal); 
    end

end

