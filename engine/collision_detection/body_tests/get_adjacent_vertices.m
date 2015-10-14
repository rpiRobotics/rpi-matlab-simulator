

% Given a vertex v in body B, returns a vector of indices of adjacent vertices.
%
% INPUT:
%       B - A Body struct 
%       v - A vertex index
%
% OUTPUT:
%       adj - A vector of adjacent vertex indices

function adj = get_adjacent_vertices( B, v )

    % For now, just use find...
%     adj = [ B.edges( find( B.edges(:,1) == v ), 2 )     % Edges where v is v_tail
%             B.edges( find( B.edges(:,2) == v ), 1 ) ];  % Edges where v is v_head

    adj = [ B.edges(  B.edges(:,1) == v , 2 )     % Edges where v is v_tail
            B.edges(  B.edges(:,2) == v , 1 ) ];  % Edges where v is v_head
        
end

