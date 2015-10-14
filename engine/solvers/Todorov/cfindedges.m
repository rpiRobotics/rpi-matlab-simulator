% find edges
%  x_edge(i) = x0 + a(i)*(x1-x0)
%  id(i) indicates which contact is reponsible for edge i
%  flgEdge(i) indicades the edge type (1: normal, 2: cone)

function [a, id, flgEdge] = cfindedges(x0, x1, mu, bnd)

EPS = 1E-8;
tol = 0;
%------------------------- initialize ----------------------------------
nc = length(x0)/3;                  % get number of contacts
x0 = reshape(x0, [3,nc]);           % make sure x0 is 3-by-nc
x1 = reshape(x1, [3,nc]);           % make sure x1 is 3-by-nc
if length(bnd)==1,                  
    bnd = bnd*ones(1,nc);           % make sure bnd is 1-by-nc
end
if length(mu)==1,                   
    mu = mu*ones(1,nc);             % make sure mu is 1-by-nc   
end
d = x1 - x0;                        % search vector

%------------------------- compute edges -------------------------------
% defined at the beginning of this file
an = (bnd-x0(1,:))./d(1,:);         % solve  x0 + a*(x1-x0) = bnd

% coefficients of cone equations for a:  A*a^2 + 2*B*a + C = 0
A = d(2,:).^2 + d(3,:).^2 - mu.^2.*d(1,:).^2;
B = d(2,:).*x0(2,:) + d(3,:).*x0(3,:) + mu.^2.*d(1,:).*(bnd-x0(1,:));
C = x0(2,:).^2 + x0(3,:).^2 - mu.^2.*(bnd-x0(1,:)).^2;

tmp = B.^2 - A.*C;                  % determinant of cone equation
flg = (tmp>=0);                     % flag real solutions
sqr = sqrt(tmp(flg));               % sqrt of determinant

ac1 = -ones(1,nc);
ac2 = -ones(1,nc);
ac1(flg) = (-B(flg)+sqr)./A(flg);   % compute two roots
ac2(flg) = (-B(flg)-sqr)./A(flg);

bad1 = (x0(1,:)+ac1.*d(1,:)>bnd);   % remove negative cone
ac1(bad1) = -1;
bad2 = (x0(1,:)+ac2.*d(1,:)>bnd);
ac2(bad2) = -1;

a = [an, ac1, ac2];                 % assemble solutions
id = [1:nc, 1:nc, 1:nc];
flgEdge = [ones(1,nc), 2*(ones(1,2*nc))];

ngood = abs(x0(1,:)-bnd)>EPS;       % not on normal edge
good = (a>=tol) & (a<=1-tol) & ...  % keep valid solutions
        isreal(a) & isfinite(a) & ...
        [ngood, ngood, ngood];
a = a(good);
id = id(good);
flgEdge = flgEdge(good);
