% compute f(x) and df/dx based on paper P2324 of "Implicit nonlinear complementarity: A new approach to contact dynamics"
%  x_n > bnd:         v_n = x_n,          f_n = 0
%  x_n < bnd:         v_n = bnd,          f_n = bnd - x_n
%  |x_t| < mu*|f_n|:  v_t = 0,            f_t = -x_t
%  |x_t| > mu*|f_n|:  v_t = x_t*(1-scl),  f_t = -x_t*scl
%    where scl = mu*|f_n|/|x_t|

function [f, flgCls, flgEdge, J1, J2, D] = ceval(x, mu, bnd)
EPS = 1E-8;                         % edge distance margin

%%%  FIX nrm==0 CASE ???
%%% ??? For the initial frame, size of x is 0, with all points sticking...
%%% How does it work here since we still have nc > 0 !!!!
%------------------------- initialize --------------------------------
nc = length(x)/3;                   % get number of contacts
x = reshape(x, [3,nc]);             % make sure x is 3-by-nc
% [m, n] = size(bnd);
% if n == 1 && m > n                  % make sure bnd is 1-by-nc
%     bnd = bnd';
% end
if length(mu)==1,                   
    mu = mu*ones(1,nc);             % make sure mu is 1-by-nc   
end
%------------------------- compute forces -----------------------------
f = zeros(3,nc);                    % allocate force
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SOME MORE THOUGHTS: 
% 1. if x(1, :) > bnd, then fN = 0 and the corresponding fT = 0; 
%    so there is no need to compute the fT any more. 
% 2. Due to this reason, the following part is limited to the cases 
%    with x(1, :) < bad. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update on f when b > xN according to equation (10) on P2324. 
%  size(x(1, :))
 
flgn = x(1,:)<bnd;                % flgn is the normal force flag, which are indicies that satisfy: b - x^N > 0 in equation(10), which is f^N > 0
f(1,flgn) = bnd(1,flgn)-x(1,flgn);  % normal force = bnd - x_n

nrm = sqrt(x(2,:).^2+x(3,:).^2);    % tangential force nrm = |x_t|
% Here scl is the s(x) in equation (12). 
scl = mu.*f(1,:)./nrm;              % scl = friction cone / |x_t|

sx  = min(1, scl(1, flgn));
% f^F(x) = -s(x) x^F
f(2,flgn) = -x(2,flgn).* sx;  % tangent force = ...
f(3,flgn) = -x(3,flgn).* sx;  %    -x_t * min(1,scl)

if nargout==1,
    return;                         % exit if the rest is not needed
end

%-------------- compute contact classes and edge flags ----------------
flgCls = zeros(1,nc,'uint8');       % 0: inactive (no force)    % 'unit8' belongs to the class of unsigned int 
flgCls(flgn) = 1 + (scl(flgn)<1);   % 1: inside friction cone   % by def of Coulomb's law
                                    % 2: outside friction cone   
                                                                         
eN = abs(bnd-x(1,:))<=EPS;          % flag normal edge         % f_n = 0;  v = 0; the normal plane (edge)     
eT = abs(mu.*f(1,:)-nrm(1,:))<=EPS; % flag friction cone edge  where |mu * fN - norm(fT)|  = 0; on the frictional cone  
                                                
flgEdge = zeros(1,nc,'uint8');      % 0: no edge
flgEdge(eN) = 1;                    % 1: normal edge  
flgEdge(eT) = 2;                    % 2: friction cone edge  

if nargout==3,
    return;                         % exit if the rest is not needed
end

%---------------------- compute Jacobians and edge normals ------------
J1 = zeros(3,3,nc);                 % allocate Jacobian 1
J2 = zeros(3,3,nc);                 % allocate Jacobian 2 (edges only)
D = zeros(3,nc);                    % allocate edge normals

for k = 1:nc                        % loop over contacts
    switch flgEdge(k),
        case 0,                     % no edge
            J1(:,:,k) = compJac(x(:,k),mu(k),f(1,k),nrm(k),flgCls(k));            
        case 1,                     % normal edge
            D(1,k) = -1; % The edge equation is f = bnd - x(1, k) = 0; Then df/dx1 = -1; normal force fn = 0
            J1(:,:,k) = compJac(x(:,k),mu(k),0,nrm(k),2);   % fn = 0; on the normal edge 
        case 2,                     % friction cone edge
            D(:,k) = [mu(k); x(2:3,k)/nrm(k)]; % The edge equation is: mu(b - xN) - sqrt(x2^2 + x3^2) = 0 % df/dx1 = -mu; df/dx2 = -x2/nrm; df/dx3 = -x3/nrm; Remove the negative sign
            D(:,k) = D(:,k) / norm(D(:,k));
            J1(:,:,k) = compJac(x(:,k),mu(k),nrm(k)/mu(k),nrm(k),2);  % On the cone, fn = nrm / mu;
            J2(:,:,k) = -eye(3);
    end          
end

%---------------------- compute Jacobian for single contact -----------
function J = compJac(x, mu, fn, nrm, cls)
switch cls,
    case 0,                         % inactive (no force)    f = 0
        J = zeros(3);
        
    case 1,                         % inside friction cone    f = b - x
        J = -eye(3);      
        
    case 2,                         % outside friction cone   f = b - S(x)x and S(x) = mu * f^N / ||x^F||
        J = -eye(3);
        % J = [df_n / dx_n      df_n / dx_t1      df_n / dx_t2    
        %      df_t1 / dx_n     df_t1 / dx_t1     df_t1 / dx_t2 
        %      df_t2 / dx_n     df_t2 / dx_t1     df_t2 / dx_t2 ];    
        J(2:3,1) = x(2:3)*mu/nrm;   % df_T / dx_n    
        J(2:3,2:3) = mu*fn/nrm* ... % df_T / dx_t 
            (x(2:3)*x(2:3)'/nrm^2 - eye(2));
end