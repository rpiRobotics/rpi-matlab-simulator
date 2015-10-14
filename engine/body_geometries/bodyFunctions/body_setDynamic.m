

% Updates the 'dynamic' field of a body struct to reflect if it is 
% dynamic or static.  Static bodies interact with non-static bodies through 
% collision, but do not move or have mass or other physical properties.  

% Calling body_setDynamic( B, false ); will make B a static body. 

function body = body_setDynamic( body, dynamic )
    body.dynamic = dynamic;  
end

