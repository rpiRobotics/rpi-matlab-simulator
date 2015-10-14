function [L, jd] = cresdir(x, A, v0, mu, bnd, d)

EPS = 1E-6;

%------------------------- compute residual and cost ---------------
f = ceval(x, mu, bnd);             % compute force
res = A*f(:) - f(:) - x + v0;       % compute residual
L = res'*res/2;                     % compute cost


%------------------------- initialize --------------------------------
nc = length(x)/3;                   % get number of contacts
x = reshape(x, [3,nc]);             % make sure x is 3-by-nc
d = reshape(d, [3,nc]);             % make sure d is 3-by-nc
if length(bnd)==1,                  
    bnd = bnd*ones(1,nc);           % make sure bnd is 1-by-nc
end
if length(mu)==1,                   
    mu = mu*ones(1,nc);             % make sure mu is 1-by-nc   
end


%----------------------- process the two sides --------------------
jd = zeros(1,2);
ee = [-EPS, EPS];
nrm = sqrt(x(2,:).^2 + x(3,:).^2);

for s = 1:2
    x1 = x + ee(s)*d;    
    flgn1 = (x1(1,:)<bnd);              % non-zero normal force
    f1 = zeros(1,nc);
    f1(1,flgn1) = bnd(1,flgn1)-x1(1,flgn1);
    nrm1 = sqrt(x1(2,:).^2+x1(3,:).^2);
    flgt1 = (mu.*f1 < nrm1) & flgn1;    % force outside cone

    Fd = zeros(3,nc);
    for k = 1:nc
        if flgn1(k) % if the normal force is non-zero
            if flgt1(k), % if the frictional force is outside the cone
                J = -eye(3);
                J(2:3,1) = x(2:3,k)*mu(k)/nrm(k);
                J(2:3,2:3) = mu(k)*f(1,k)/nrm(k)* ...
                    (x(2:3,k)*x(2:3,k)'/nrm(k)^2 - eye(2));
                Fd(:,k) = J*d(:,k);
            else
                Fd(:,k) = -d(:,k);
            end
        end
    end
    
    jd(s) = res'*(A*Fd(:)-Fd(:)-d(:));
end
