function [L, res, J, B, flgB] = cresidual(x, A, v0, mu, bnd)

EPS = 1E-10;

% no Jacobian needed
if nargout<3,
    f = ceval(x, mu, bnd);
    f = f(:);
    % v = v0 + A*f;   res = A*f + v0 - v  (v = f + x)
    res = A*f + v0 - f - x;
    L = res'*res/2;
    return;
end

% Jacobian needed
% f ----- forces ;  flaCls ------flag classes ;    flgEdge ------- flag
% edges;  D ------- edge normals 
[f, flgCls, flgEdge, J1, J2, D] = ceval(x, mu, bnd);
f = f(:);
res = A*f + v0 - f - x;   % Compute residual using equation (15)  which is res = (A - I) f - x + v0
L = res'*res/2;           % Compute 1/2 residual^2  

nc = length(x)/3;
I = eye(3*nc);
J = A - I;   % coefficient of f in the res(idual) since res = (A - I) f - x + v0

BB = zeros(3,2,nc);  
flgB = zeros(1,nc); 
for k = 1:nc
    ii = 3*(k-1)+1:3*k;   % ii is the index range for later use
    if flgEdge(k)==0,     % This is the case that there is NO EDGE, for this case D is zero, and J1 is meaningful
        jj = J1(:,:,k);   % jj is the 3 by 3 Jacobian for each contact 
    else % There IS edge
        g1 = (J(:,ii)*J1(:,:,k) - I(:,ii))'*res;  % g = J' * r; according to equation below equation (26)
        g2 = (J(:,ii)*J2(:,:,k) - I(:,ii))'*res;  
        del1 = g1'*D(:,k);           % From equation (25); delta1 = r'J1*D  = (J1' * r)' * D = g1' * D            
        del2 = -g2'*D(:,k);          % -g2'; using the opposite direction by adding "-"

        if del1 < 0 && del2 < 0,     % use larger gradient
            if norm(g1) > norm(g2)  
                jj = J1(:,:,k);      % use J1 here since norm(g1) is larger
            else
                jj = J2(:,:,k);       
            end
        elseif del1 < 0,                
            jj = J1(:,:,k);     
        elseif del2 < 0,
            jj = J2(:,:,k); 
        else                        % no improvement: use mixture
            al = del2/(del1+del2);    
            al = min(1, max(0, al));
            jj = al*J1(:,:,k) + (1-al)*J2(:,:,k);  % equation (26) in the paper 
            if flgEdge(k)==1,  % Normal edge
                BB(:,:,k) = [0, 0; 1 0; 0 1];      % this is the B_plane in euqation (29)
                flgB(k) = 1;   % Normal edge
            else
                xx = x(ii);
                BB(:,:,k) = [xx(1)-bnd(k), 0; xx(2), -xx(3); xx(3), xx(2)];  % this is the B_cone in equation (29)
                flgB(k) = 2;   % frictional edge; which is the frictional cone
            end
        end
    end
    J(:,ii) = J(:,ii)*jj; 
end

J = J - I;
if any(flgB>0),    % The normal edge and frictional edge
    num = sum(flgB>0);
    % The matrix B in equation (28)
    B = sparse(3*nc, 3*nc-num);    
    adr = 0;
    for k = 1:nc   % Construct the B matrix, only when flgB(k) > 0 there is nonzero element.
        if flgB(k)>0,  % There is EDGE
            B(3*(k-1)+1:3*k, adr+1:adr+2) = BB(:,:,k);     
            adr = adr+2;
        else  % There is no edge; THEN (adr+1 : adr+3)
            B(3*(k-1)+1:3*k, adr+1:adr+3) = eye(3);            
            adr = adr+3;
        end
    end
else  % if there is NO active edge; from description above equation (29)
    B = speye(3*nc);
end
