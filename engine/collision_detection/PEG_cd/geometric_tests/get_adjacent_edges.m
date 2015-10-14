
% Given a vertex v in body B, returns a vector of indices of adjacent edges.
%
% INPUT:
%       B - A Body struct 
%       v - A vertex index
%
% OUTPUT:
%       adj - A vector of adjacent edge indices
function adj = get_adjacent_edges( B, v )

    adj = [find(B.edges(:,1)==v)' find(B.edges(:,2)==v)'];

end

