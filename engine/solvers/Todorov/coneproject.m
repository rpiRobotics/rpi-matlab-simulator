% project point on friction cone
% also here update on x; 
% the input "p" is x before update
function x = coneproject(p, mu, bnd)
X = zeros(3,2);
dst = zeros(1,2);

tmp = mu.*sqrt(p(2:3)'*p(2:3));   % mu * sqrt(p2^2 + p3^2) 
X(1,1) = (p(1) + mu.^2*bnd + tmp)/(1+mu.^2);
X(1,2) = (p(1) + mu.^2*bnd - tmp)/(1+mu.^2);

for k = 1:2
    if X(1,k) < bnd,
        tmp = mu.^2*(X(1,k)-bnd);
        X(2:3,k) = p(2:3) * tmp / (tmp+X(1,k)-p(1)); 
        dst(k) = norm(p-X(:,k));
    else
        dst(k) = inf;
    end
end

[d, ind] = min(dst);

if isfinite(d),
    x = X(:,ind);
else
    x = [bnd; 0; 0];
end

%{
disp([x p]);

% plot
figure(1);
clf;
ng = 31;
[G1, G2] = ndgrid(linspace(-5,5,ng),linspace(-5,5,ng));
mesh(G1,G2,bnd-sqrt(G1.^2+G2.^2)/mu,'facecolor','none','edgecolor','b');
hold on;

plot3([x(2) p(2)],[x(3) p(3)],[x(1) p(1)],'r.-','linewidth',2);
plot3(x(2),x(3),x(1),'ko','markersize',10);

axis image;
%}