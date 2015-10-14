% minimize contact error, edge-aware, cubic search
function [x, res, L, it, flag, STAT] = cminimize(x, A, v0, mu, bnd)
% optimizer parameters
itmax = 50;
restol = 1E-6;
gradtol = 1E-15;
dectol = 0.5;
lambda = 1E-6;
lafac = 5;
lamin = 1E-10;
trust = 5;

% get sizes
sz = size(A,1);  % size of contact force  [n t o] 3 directions
nc = length(bnd);       

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initially, for the very first step, the size of x is 0 by 0 iff the contact number has changed 
% and also  the size of v is also 0 by 0.  But after update, the size keeps changing. 

% For each frame, the size of x is different.  

% initialize assuming all colliding contacts stick
%if isempty(x),
    v = v0;
    for k = 1:nc
        if v0(3*(k-1)+1) <= bnd(k),  % bnd here is nc by 1
            v(3*(k-1)+1) = bnd(k);
            v(3*(k-1)+(2:3)) = 0;
        end
    end
    % make sure all contacts are sticking, which is v = 0; 
    v(1:3:end) = bnd;   % make sure the normal contact velocity is 0 !
    f = pinv(A)*(v-v0);  
    x = v-f; 
%end

STAT = zeros(1,8);
% major iteration
for it = 1:itmax    
    % L is 1/2 * res' * res
    % res is residual 
    % J is the Jacobian of the residual; 
    % B is the B matrix in equation (28) (29), which is used to find xC xN
    % flgB =0 --- no edge; flgB = 1 -- normal edge;   flgB = 2 --- friction edge; 
    
    [L,res,J,B,flgB] = cresidual(x,A,v0,mu,bnd);  % evaluate current point by computing its residual
     
    % use to check whether the solution converges 
    grad0 = J'*res;                             % unrestricted gradient equation (26) below
    
    if norm(res) <= restol,                     % SUCCESS - small residual
        flag = 0;
        %STAT = zeros(1,8);
        return;
    elseif norm(grad0) <= gradtol,              % FAILURE - small gradient
        flag = 1;
        %STAT = zeros(1,8);
        return;
    elseif it >= itmax,                         % FAILURE - max iterations
        flag = 2;
        %STAT = zeros(1,8);
        return;
    end
    
    JB = J*B;                                   % restricted Jacobian
    grad = JB'*res;                             % restricted gradient
   
    % equation (21) to get the matrix function; Gauss-Newton approximation
    % to the Hessian of l
    H = JB'*JB + lambda*eye(size(B,2));         % restricted Hessian
    
    % equation (22); xN = x - inv(H)*g;  
    del = -H\grad;                              % restricted Newton delta
    
    
    xNP = x + B*del;                            % non-restricted NP
    xNP = project(xNP,mu,bnd,flgB);             
    LNPpred = max(0, L + (grad'*del)/2);        % predict Newton error 
    LNP = cresidual(xNP,A,v0,mu,bnd);          % evaluate Newton error
    if (L-LNP) > dectol*(L-LNPpred),            % if sufficient improvement
        x = xNP;                                %  accept Newton point
        lambda = max(lamin, lambda/lafac);      %  decrease lambda
        STAT(1) = STAT(1)+1;
        continue;
    end
    
    lambda = lambda*lafac;                      % increase lambda
    % equation (28) xC
    xCP = x - ...
        B*grad*(grad'*grad)/(grad'*H*grad);     % non-restricted CP
    xCP = project(xCP,mu,bnd,flgB);
    
    scl = trust/norm(x-xCP);                    % trust region for Cauchy
    if scl<1,
        xCP = x + scl*(xCP-x);
    end
  
    scl = trust/norm(xNP-xCP);                  % trust region for Newton
    if scl<1,
        xNP = xCP + scl*(xNP-xCP);
    end
    
    a1 = cfindedges(x, xCP, mu, bnd);           % find edges in [x, xCP]
    a2 = cfindedges(xCP, xNP, mu, bnd);         % find edges in [xCP, xNP]
    a1 = sort(a1);  % put a1 into increasing order 
    a2 = sort(a2);  % put a2 in increasing order
    sz1 = length(a1);
    sz2 = length(a2);
    szlist = 3+sz1+sz2;
    
    X = zeros(sz,szlist);                       % allocate list of points
    X(:,1) = x;                                 % add x
    for k = 1:sz1   % edges in [x, xCP]
        X(:,1+k) = x + a1(k)*(xCP-x);           % add edges in [x, xCP]
    end
    X(:,2+sz1) = xCP;                           % add xCP
    for k = 1:sz2   % edges in [xCP, xNP]
        X(:,2+sz1+k) = xCP + a2(k)*(xNP-xCP);   % add edges in [xCP, xNP]
    end
    X(:,end) = xNP;                             % add xNP
    
    for k = 1:szlist
        % project on the the normal edge or the frictional edge; especially
        % for the coneProject(), which implements the last paragraph before
        % part E: to project the points on the (cone) edge surface;
        X(:,k) = project(X(:,k),mu,bnd,flgB);
    end
    
    Llist = zeros(1,szlist);                    % allocate list errors
    Jlist = zeros(2,szlist);                    % allocate list derivs
    d = (xCP-x)/norm(xCP-x);                    % dir = xCP-x
    for k = 1:szlist
        if k==3+sz1,
            d = (xNP-xCP)/norm(xNP-xCP);        % dir = xNP-xCP
        end
        [Llist(k), Jlist(:,k)] = ...            % compute L and deriv
            cresdir(X(:,k),A,v0,mu,bnd,d);
    end
    [dum, jd] = cresdir(X(:,2+sz1),A,v0,mu,bnd,d);
    Jlist(2,2+sz1) = jd(2);                     % fix CP_plus deriv
    
    [Lbest, ind] = min(Llist);                  % find minimum so far
    Xbest = X(:,ind);
    if ind>1,
        if ind==szlist,
            change = 2;     % newton
        elseif ind==2+sz1,
            change = 3;     % cauchy
        elseif ind==2,
            change = 4;     % first edge after x0
        elseif ind<2+sz1,
            change = 5;     % between x0 and cauchy
        else
            change = 6;     % between cauchy and newton
        end
    else
        change = 0;
    end
    
    
    for k = 1:szlist-1                          % find min in each segment    
        [LL, xx] = polysearch(X(:,k),Llist(k),Jlist(2,k),...
            X(:,k+1),Llist(k+1),Jlist(1,k+1), A,v0,mu,bnd, k==1);
        
        if LL < Lbest,                          % update minimum
            Lbest = LL;
            Xbest = xx;
            if k<2+sz1,
                change = 7; % between x0 and cauchy
            else
                change = 8; % between cauchy and newton
            end
        end
    end
    
    
    if change,                                  % improvement: assign
        x = Xbest;
        STAT(change) = STAT(change)+1;
    else
        flag = 3;                               % FAILURE - no improvement
        %STAT = zeros(1,8);
        return;
    end
end
    

%---------------------- project on active edge --------------------------
function xp = project(x,mu,bnd,flgB)
xp = x;
nc = size(x)/3;
for k = 1:nc
    if flgB(k)==1,     % Normal edge where fN = 0 v = 0; --->  x = 0
        xp(3*k-2) = bnd(k);
    elseif flgB(k)==2, % Frictional edge, where fN > 0 and mu * fN = sqrt(x2 ^2 + x3 ^2)
        xp(3*(k-1)+1:3*k) = coneproject(xp(3*(k-1)+1:3*k), mu, bnd(k)); 
    end
end


%---------------------- fit cubic or quadratic, find minimum -------------
function [L, x] = polysearch(x0,L0,J0, x1,L1,J1, A,v0,mu,bnd, persist)

atol = 1E-7;                                    % min leading coef.
jtol = 1E-10;                                   % min deriv
dtol = 1E-5;                                    % min distance from x0,x1
maxit = 5;

if L0<=L1,                                      % find best so far
    L = L0;
    x = x0;
else
    L = L1;
    x = x1;
end


for it = 1:maxit
    nrm = norm(x1-x0);

    d = L0;                                     % fit cubic:
    c = J0*nrm;                                 %  a*x^3 + b*x^2 + c*x + d
    u = L1 - c - d;
    v = J1*nrm - c;
    b = 3*u - v;
    a = v - 2*u;

    xx = [];
    tmp = b^2 - 3*a*c;                          % determinant of cubic'                          
    if tmp>=0 && abs(a)>atol,                   % check for minimum
        xx = (sqrt(tmp)-b)/3/a;                 % minimize cubic
        if xx<dtol || xx>1-dtol,
            xx = [];                            % minimum outside (0,1)
        end
    end

    if isempty(xx),                             % fit quadratic:
        a = (J1-J0)*nrm/2;                      %  a*x^2 + b*x + c
        b = L1-L0 - a;
        c = L0;
        
        if a>atol,                              % check for minimum
            xx = -b/2/a;                        % minimize quadratic
            if xx<dtol || xx>1-dtol,
                xx = [];                        % minimum outside (0,1)
            end
        end
    end

    if isempty(xx),                             % polysearch failure
        if persist,
            xx = 0.5;                           % use midpoint
        else
            return;                             % give up
        end
    end

    xnew = x0 + xx*(x1-x0);                     % evaluate new point
    [Lnew, tmp] = cresdir(xnew, A,v0,mu,bnd, (x1-x0)/nrm);
    Jnew = tmp(1);

    if Lnew<L,                                  % check for new best
        L = Lnew;
        x = xnew;
    end

    if abs(Jnew)<jtol,                          % stop if deriv is small
        return;
    end

    if Jnew<0,                                  % decreasing: [xnew, x1]
        x0 = xnew;
        L0 = Lnew;
        J0 = Jnew;
    else                                        % increasing: [x0, xnew]
        x1 = xnew;
        L1 = Lnew;
        J1 = Jnew;
    end
end
