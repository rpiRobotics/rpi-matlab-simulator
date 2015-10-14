
% Given a vertex v in body B, returns a vector of indices of adjacent faces.
%
% INPUT:
%       B - A Body struct 
%       v - A vertex index
%
% OUTPUT:
%       adj - A vector of adjacent face indices
function adj = get_adjacent_faces( B, v )

    adj = [find(B.faces(:,1)==v)' find(B.faces(:,2)==v)' find(B.faces(:,3)==v)'];

end

