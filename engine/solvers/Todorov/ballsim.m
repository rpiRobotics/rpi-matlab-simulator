% simulate ball collisions
function [LL, IT, NNC, STAT, PP] = ballsim

%----------------- define parameters -----------------------------------
N = 10;                      % number of balls
R = .1+.05*rand(1,N);       % ball radius (m)
Dense = 500;                % material density (kg/m^3)
SN = 20;                    % number of sphere patches
NCMAX = 20;                 % max number of contacts to render
MU = 1;                     % friction coefficient

BOX = 2*[0.5, 0.5, 0.5];      % box size (+/- m in each direction)
GRAV = [-9.8, -9.8, -9.8];        % gravity vector
DT = 0.005;                  % time step (sec)

P = 0.6*randn(3,N);         % 3D positions (m)
Q = zeros(4,N);             % quaternion orientations
Q(1,:) = 1;
V = randn(3,N);             % translation velocities
W = randn(3,N);             % rotation velocities
V(3,:) = V(3,:)+1;          % z-directional velocity plus 1

% {
% uncomment to stack the initial ball positions
% P = zeros(3,N);
% P(3,:) = ((1:N)-N/2)/N*0.7;
% dst = abs(P(3,2)-P(3,1));
% R = ones(1,N)*(dst/2+0.001);
% V = zeros(3,N);
% W = zeros(3,N);
% }

%------------------ make inertia matrix --------------------------------
m = 4/3*pi*R.^3 * Dense;   % masses:    1 by N matrix 
i = 2/5*m.*R.^2;           % inertias:  1 by N matrix

tmp = [ones(3,1)*m; ones(3,1)*i];      
M = tmp(:);                % generalized mass: 6 by N matrix

%------------------ create figure and 3d objects -----------------------
figure(1);
clf;
[X{1},X{2},X{3}] = sphere(SN);  % SN is the number of sphere patches

for k = 1:N         % N is the number of balls
    P(:,k) = min(BOX'-R(k), max(-BOX'+R(k), P(:,k)));   % make sure the ball is inside the box
    for i = 1:3
        x{i} = X{i}*R(k) + P(i,k);
    end
    h(k) = surf(x{1},x{2},x{3},rem(5*abs(x{3}),1));   % SPHERE handle
    set(h(k), 'facealpha', 0.5);   % facealpha, is a scalar between 0 and 1 that controls the transparency of all the faces of the object
    shading flat; % each mesh line segment and face has a constant color determined by the color value at the endpoint of the segment
                  % or the corner of the face that has the smallest indices
    hold on;
end

% hc is the contact handle
for k = 1:NCMAX   % max number of contacts to render 
    hc(k) = surf(x{1},x{2},x{3},ones(SN+1,SN+1));  % last argument to define color
    set(hc(k), 'visible','off');    
end

camlight;   % create a light right and up from camera; the same as camlight('right');
box on;
grid off;
set(gca,'xtick',[],'ytick',[],'ztick',[]);
axis image;
axis([-BOX(1), BOX(1), -BOX(2), BOX(2), -BOX(3), BOX(3)]);
set(gcf,'renderer','zbuffer','doublebuffer','on','color',[0 0 0]);
view([-25,25]);
dataname = {'XData', 'YData', 'ZData'};
ht = text(-0.4,0.4,0.45,'text');

%------------------------ run simulation loop ---------------------------
RUN = 200;
xc = [];
BAD = 0;
GOOD = 0;
LL = zeros(1,RUN);
IT = zeros(1,RUN);
STAT = zeros(1,8);
PP = zeros(2,N,RUN);
for cnt = 1:RUN,
    tic;
    PP(1,:,cnt) = P(3,:);  % z position 
    PP(2,:,cnt) = V(3,:);  % z velocity
    
    %--------------------------------------- find contacts
    CON = [];
    for k = 1:N
        con = boxsphere(BOX, P(:,k),R(k));   % The contact point with the wall
        if ~isempty(con),
            sz = size(con,2);
            CON = [CON, [[k;0]*ones(1,sz); con]];  % [k; 0]*ones(1, sz) is kind of repmat to form matrix; the index of wall is 0; the index of sphere is k
        end
        
        for k1 = k+1:N
            con = spheresphere(P(:,k),R(k), P(:,k1),R(k1));        
            if ~isempty(con),
                CON = [CON, [k; k1; con]];
            end
        end
    end
    %--------------------------------------- construct Jacobian
    NC = size(CON,2);    % number of contact
    if NC>0,
        J = zeros(3*NC, 6*N);
        for k = 1:NC
            T = maketangent(CON(6:8,k)); % get the tangent T = [n, t1, t2]
            adr = 3*(k-1)+1:3*k;         % current active index range 
            for i = 1:2
                if CON(i,k)>0,
                    bid = CON(i,k);      % find the index of bodies that form the contacts
                    adr1 = 6*(bid-1)+1:6*(bid-1)+3;
                    sgn = 1 - 2*(i-1);   % take the 1st body as positive 1 while the 2nd body as negative 1
                    J(adr, adr1) = sgn*T';  % [n, t1, t2]
                    J(adr, adr1+3) = -sgn*T'*crosmat(P(:,bid)-CON(3:5,k));  %[rXn  rXt1  rXt2]
                end
            end
        end
    end
    %--------------------------------------- solve minimization problem
    g = [GRAV'*m; zeros(3,N)];  % forces and torques  6 by N matrix
    g = g(:);
    v = [V; W];
    v = v(:);
    bias = M.*v + DT*g;         % DT is time step  Then (Minv * bias = v0)
    
    impulse = zeros(6*N,1);     % 6*N by 1 impulse 
    L = eps;                    % L = 1/2 * res' * res;
    it = 0;
       
    if NC>0,
        JMi = J .* (ones(3*NC,1) * (1./M'));   % Jacobian Mass inverse % ones(3*NC, 1) is used to modify the mass matrix size 
        A = JMi * J';  % This is A matrix from the paper, which is JMinvJ';
        v0 = JMi * bias;  % v0 is contact velocity, and v0 = K Minv c;  v0= J * general velocity
        
        if length(xc) ~= 3*NC,
            xc = [];
        end
        xcold = xc; 
        %----------------------------------solver part
        % xc  ------------------ the x in the equation
        % res ------------------ the residual 
        % L   ------------------ 1/2 * res' * res; the squared residual
        % it  ------------------ number of iterations 
        % flag ----------------- 0 ----> success, small residual;
        %                        1 ----> fail, small gradient
        %                        2 ----> fail, max iterations
        % ST   -----------------  Number of sufficient improvement
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [xc, res, L, it, flag, ST] = cminimize(xc, A, v0, MU, 0);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        STAT = STAT + ST;  % STAT size 1 by 8
        fc = ceval(xc, MU, 0); % compute the fc, which is the force at contact points
        
        impulse = J' * fc(:);  % This is the generalized impulse in world frame, J' * contact impulse = generalized impulse
        %disp(it);
        
        if flag>0,  % BAD case
            BAD = BAD+1;
        else
            GOOD = GOOD+1;
        end
    end
    LL(cnt) = L;    
    IT(cnt) = it;   % number of iterations
    NNC(cnt) = NC;  % number of contact points in each simulation frame; 
    
    v1 = (bias + impulse) ./ M;  % impulse is the genearlized impulse in world frame;  
    v1 = reshape(v1, [6,N]); % reshape the v1 matrix and return 6 by N matrix whose element taken v1;
    V = v1(1:3,:);           % contact velocity
    W = v1(4:6,:);           % contact angular velocity
    %--------------------------------------- advance simulation state
    P = P + DT*V;   % spheres position update 
    % Update the orientation by update the quaternions
    for k = 1:N     
        D = dotmat(Q(:,k));
        Q(:,k) = Q(:,k) + DT*D*W(:,k);    
        Q(:,k) = Q(:,k) / norm(Q(:,k));
    end
    %--------------------------------------- render simulation state
    title(sprintf('%d: %.3f %.2f %d',cnt,toc,log10(L),NC));
    str = sprintf('Frm %d,  Contact %d,  Iter %d',cnt, NC,it);
    set(ht,'string',str);
    
    % update the SPHERE handle
    for k = 1:N
        RM = rotmat(Q(:,k));
        XX = R(k) * RM * [X{1}(:), X{2}(:), X{3}(:)]';        
        for i = 1:3
            set(h(k),dataname{i},P(i,k) + ...
                reshape(XX(i,:), [SN+1, SN+1]));
        end
    end
    
    % update the CONTACT handle
    %--------------------------------------- render contacts
    for k = 1:NCMAX
        set(hc(k),'visible','off');
    end
    for k = 1:min(NCMAX, size(CON,2))
        XX = 0.02 * [X{1}(:), X{2}(:), X{3}(:)]';        
        for i = 1:3
            set(hc(k),dataname{i},CON(2+i,k) + ...
                reshape(XX(i,:), [SN+1, SN+1]));
        end
        set(hc(k),'visible','on');
    end
    drawnow;
end


%---------------------- Jacobian functions ---------------------------
function T = maketangent(n)

[dummy,i] = min(abs(n));
t1 = zeros(3,1);
t1(i) = 1;
t2 = cross(n,t1);
t2 = t2 / norm(t2);
T = [n, t1, t2];


function C = crosmat(v)

C = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];


%---------------------- collision functions ---------------------------
function con = spheresphere(p1, r1, p2, r2)
con = [];
dst = norm(p1-p2);
if dst<=r1+r2,
    cdst = 0.5*(r2-r1+dst); % This is the mid point of the penetration depth, which is regarded as the contact point; stability computation
    c = p2 + (p1-p2)*cdst/dst; % This is the mid point coordinate, as contact coordinate
    n = (p1-p2)/dst;  % normal 
    con = [c; n];     % c is the contact point, n is the normal
end


function con = boxsphere(B, p, r)

con = [];
for i = 1:3
    if p(i)-r <= -B(i),  % If the sphere is out of the box (left down front), checking x y z direction
        c = p;
        c(i) = p(i)-r;
        n = zeros(3,1);
        n(i) = 1;
        con = [con, [c;n]];
    end
    if p(i)+r >= B(i), % If the sphere is out of the box (right up back), checking x y z direction
        c = p;
        c(i) = p(i)+r;
        n = zeros(3,1);
        n(i) = -1;
        con = [con, [c;n]];
    end
end


%---------------------- quaternion functions --------------------------
% convert the quaternion to the rotation matrix R
function R = rotmat(q)

R = zeros(3,3);

R(1,1) = q(1)^2 + q(2)^2 - 0.5; 
R(2,2) = q(1)^2 + q(3)^2 - 0.5;
R(3,3) = q(1)^2 + q(4)^2 - 0.5;

R(1,2) = q(2)*q(3)+q(1)*q(4);
R(2,1) = q(2)*q(3)-q(1)*q(4);

R(1,3) = q(2)*q(4)-q(1)*q(3);
R(3,1) = q(2)*q(4)+q(1)*q(3);

R(2,3) = q(3)*q(4)+q(1)*q(2);
R(3,2) = q(3)*q(4)-q(1)*q(2);

R = 2*R;


function D = dotmat(q)

D = [-q(2), -q(3), -q(4);...
      q(1), -q(4),  q(3);...
      q(4),  q(1), -q(2);...
     -q(3),  q(2),  q(1)];
 
 D = D/2;
 