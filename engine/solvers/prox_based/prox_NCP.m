function [z, newNU, itemize_error, contact_counting]  = prox_NCP(Minv, G, U, Pext, PSI, NU, h)


% Implemented following red book: page 54
% refered to Nutzi's IDETC paper 


% Input: 
% 
% Minv  --------- (6*nb)X(6*nb) Inverse of the Mass matrix M 
% G     --------- (6*nb)X(3*nc) Jacobian matrix   Gn1 Gf1 Gn2 Gf2 
% U     --------- (nc)X(nc), diagonal matrix of frictional coefficient
% Pext  --------- (6*nb)X 1, external impulse 
% PSI   --------- (nc)X 1,   gap distance 
% NU    --------- (6*nb)X1,  generalized velocity 
% h     ---------- scalar, time step 
% 

% Output: 
% z     ----------  z = [pn; pf]
% newNU ----------  newNU is the new generalized velocity 

% for update 
% solution.newNU = newNU;

 
nc = size(G, 2) / 3;
% Only two frictional directions for the cone  
Gn = G(:, 1:3:end);
Gf = G(:, setxor(1:end, 1:3:end)); 
MinvG = Minv*G;
MinvGn = MinvG(:, 1:3:end);
MinvGf = MinvG(:, setxor(1:end, 1:3:end)); 
MinvPext = Minv * Pext;

Delassus = G' * MinvG;
R = zeros(2*nc, 1);

max_iter = 200;
tol = 1e-4;

for i = 1 : 3*nc 
    if(mod(i, 3) == 1)
        R(i) = 1 / Delassus(i, i);
    else if (mod(i, 2) == 2)
            R(i) = max(1/Delassus(i,i), 1/Delassus(i+1, i+1));
        else
            R(i) = max(1/Delassus(i, i), 1/Delassus(i-1, i-1));
        end
    end
end

R_n = R(1:3:end);
R_f = R(2:3:end);
 
 
old_err = Inf;
b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc);
pn = zeros(nc, 1);
pn_ellp1 = zeros(nc, 1);
pf = zeros(2*nc, 1);
pf_ellp1 = zeros(2*nc, 1); 
warm_start = '';
 
switch warm_start
    case 'Lemke'
        Anorm       = Gn'*MinvGn;
        bnorm       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
        pn = lemke(Anorm, bnorm, pn);
    case 'quad_program'
        Anorm       = Gn'*MinvGn;
        bnorm       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
        
        %opts  = optimset('Algorithm', 'active-set', 'Display', 'off');
        opts  = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
        cons_A = [-1*eye(length(bnorm));  -Anorm];
        cons_b = [zeros(length(bnorm), 1);  bnorm];
        pn  = quadprog(2*Anorm, bnorm, cons_A, cons_b, [], [], [], [], [], opts);
    otherwise
        %disp('No warm start metric is used')
end

% one iteration is a loop over all the contacts
for iter = 1 : max_iter  
    NU_current = NU + MinvGn*pn + MinvGf*pf + Minv*Pext;         
 
    rho_n = PSI/h + Gn'*NU_current;
    rho_f = Gf' * NU_current;
    
    % Solve normal here: 
    Anorm       = Gn'*MinvGn;
    bnorm       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
    pn = lemke(Anorm, bnorm, pn);
    
    for i = 1 : nc
        % update and project friction 
         pn_ellp1(i,1) = max(0, pn(i, 1)-R_n(i)*rho_n(i));
        % manually make pn bounded 
         pn_ellp1(i, 1) = min(100, pn_ellp1(i, 1));
         
        % update friction
        for j = 1 : 2
            cuindex = 2*(i-1)+j;
            pf_ellp1(cuindex, 1) = pf(cuindex, 1) - R_f(i)*rho_f(cuindex, 1);
        end
        
        % Project friction
        pm = pn_ellp1(i, 1);
        ps = pf_ellp1(2*(i-1)+1); pt = pf_ellp1(2*(i-1)+2);
        if (pm == 0)
            ps = 0; pt =0;
        else
            mu = U(i, i);
            cone_radius = mu * pm;
            fric_length = sqrt(ps^2 + pt^2);
 
            if (fric_length > cone_radius)
                ps = (ps/fric_length)*cone_radius;
                pt = (pt/fric_length)*cone_radius;
            end
        end
        pf_ellp1(2*(i-1)+1) = ps;
        pf_ellp1(2*(i-1)+2) = pt;
    end
     
 
    err = norm(pn_ellp1-pn) + norm(pf_ellp1-pf);
    
    if( norm(old_err -  err) < tol  &&  norm(err)  < tol)
        break;
    else 
        old_err = err;
        pn = pn_ellp1;
        pf = pf_ellp1;
    end 
end

z = [pn_ellp1; pf_ellp1]; 
newNU = NU + MinvGn*pn_ellp1 + MinvGf*pf_ellp1 + Minv*Pext;   

% Error calculating 
% itemize_error; % size: 1 X 5 [norml_error, slide_align, stick_residual, stick_cone_satisfy, slide_cone_satisfy ]
% contact_counting; % size: 1 X 3 [slide_badAlign, stick, slide]

bad_slide_align = 0;
stick_num = 0;
slide_num = 0;
stick_residual = 0;
slide_cone_satisfy = 0;
slide_align = 0;

normal_error = abs(PSI' * pn_ellp1) / nc;

all_relative_vel = Gf'*newNU; 

for i = 1 : nc
     
    current_friction_magnitude = norm([pf_ellp1(2*(i-1)+1),  pf_ellp1(2*(i-1)+2)]);
    cone_radius = U(i, i) * pn_ellp1(i);
    
    
    
    if (current_friction_magnitude < cone_radius || cone_radius < 1e-12)
        stick_num = stick_num + 1;
        current_con_sliding_speed  = norm(all_relative_vel(2*(i-1)+1:2*(i-1)+2, 1));
        stick_residual = stick_residual + current_con_sliding_speed/nc;
    else        
        slide_num = slide_num + 1;
        
        if (current_friction_magnitude > cone_radius)
            curr_error = (current_friction_magnitude - cone_radius) / (cone_radius * nc);
            slide_cone_satisfy = slide_cone_satisfy + curr_error;
        end
        
        relative_vel_dir = all_relative_vel(2*(i-1)+1:2*(i-1)+2, 1); 
        relative_vel_dir = relative_vel_dir / norm(relative_vel_dir);
        
        friction_dir = pf_ellp1(2*(i-1)+1:2*(i-1)+2, 1);
        friction_dir = friction_dir / norm(friction_dir);
         
 
        cos_angle = relative_vel_dir' * friction_dir;
        if(cos_angle > 0)
            bad_slide_align = bad_slide_align + 1;
        end
  
        angle = atan(sqrt(1-cos_angle^2) / cos_angle);
      
        
        if angle > 0
            % pi 
            slide_align = slide_align + (pi - angle) /(pi * nc);
        else 
            slide_align = slide_align + (angle + pi) / (pi * nc);
        end
        
    end
end

itemize_error = [normal_error, slide_align, stick_residual, slide_cone_satisfy];
contact_counting = [bad_slide_align, stick_num, slide_num]; 



end


function b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc)
b = [ Gn'*(NU + MinvPext) + PSI/h;     
      Gf'*(NU + MinvPext);
      zeros(nc,1) ];    
end
