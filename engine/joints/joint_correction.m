%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% This function updates bodies positions and velocities of simulator sim to
% correct for joint constraint violations. 

function sim = joint_correction( sim )

    epsPosition = 10^-5;                  % Joint position error epsilon  
    epsVelocity = 10^-5;                  % Joint velocity error epsilon 
    maxIters = 25;            
    nj = length(sim.joints);              % Number of joints 
    %nb = sim.num_activeJointBodies;       % Number of bodies involved
    %njc = sim.num_jointConstraints;       % Number of joint constraints
    h = sim.h;                            % Simulation time-step 

    if sim.jointCorrection

      %% For every joint, record values BEFORE update is applied, i.e. at time t0
      for j=1:nj
         J = sim.joints(j);
         [Gb, ~, ~] = joint_getCorrectionValues(sim,j); % J.getJointCorrectionValues();
         J.bender.Gb_t0 = Gb; 
         J.bender.Minv = inv(J.getMassInertiaMatrix());
         J.bender.MinvGb0 = J.getMassInertiaMatrix() \ Gb;  
         J.update(); 
      end


      %% Position correction 
      PiterCount = 0;
      Cmax = inf; 
      Piter = 0;
      Pcorrected = false; 
      while Piter < maxIters && Cmax > epsPosition
          Piter = Piter+1; 
          Cmax = 0; 
          totalPerr = 0;
          for j=1:nj
                J = sim.joints(j); 
                J.update(); 

                b1 = J.body1;
                b2 = J.body2; 

                Gb_t0 = J.bender.Gb_t0; 
                Minv = J.bender.Minv; 
                MinvGb0 = J.bender.MinvGb0; 

                % Calculate constraint error
                [~, C_t1, Cdot] = J.getJointCorrectionValues();
                if norm(C_t1) > Cmax, Cmax = norm(C_t1); end
                if norm(C_t1) > epsPosition, Pcorrected = true; end
                
                pi = 0; 
                        JERR = [];
                while norm(C_t1) > epsPosition && pi < 50
                    PiterCount = PiterCount + 1;
                    pi = pi+1;

                    %dp = -inv(Gb_t0' * MinvGb0) * ((C_t1/h) + Cdot);  % Cdot seems to cause trouble 
                    dp = -inv(Gb_t0' * MinvGb0) * (C_t1/h); % + Gb_t0'*J.Fext();
                    dnu = MinvGb0 * dp; 

                    % Update positions & velocities
                    if b1.static
                        b2.u = b2.u + dnu(1:3) * h;
                        b2.nu = b2.nu + dnu(1:6); 
                        
                        b2.quat = b2.quat + qtdq(b2.quat) * dnu(4:6)*h;
                        b2.quat = b2.quat / norm(b2.quat); 

                    elseif b2.static
                        b1.u = b1.u + dnu(1:3) * h; 
                        b1.nu = b1.nu + dnu(1:6);
                        
                        b1.quat = b1.quat + qtdq(b1.quat) * dnu(4:6)*h;
                        b1.quat = b1.quat / norm(b1.quat); 

                    else
                        b1.u = b1.u + dnu(1:3) * h; 
                        b2.u = b2.u + dnu(7:9) * h;

                        b1.nu = b1.nu + dnu(1:6);
                        b2.nu = b2.nu + dnu(7:12); 
                        
                        
                        b1.quat = b1.quat + qtdq(b1.quat) * dnu(4:6)*h;
                        b1.quat = b1.quat / norm(b1.quat); 
                        b2.quat = b2.quat + qtdq(b2.quat) * dnu(10:12)*h;
                        b2.quat = b2.quat / norm(b2.quat); 
                    end

                    % Recalculate constraint error
                    J.update(); 
                    [~, C_t1, ~] = J.getJointCorrectionValues();   
                    if norm(C_t1) > Cmax, Cmax = norm(C_t1); end

                    %C_t1'
                                    %disp(['   ' num2str(pi) ': ' num2str(norm(C_t1)) ]);
                    JERR = [JERR; norm(C_t1)];
                    %figure(5); 
                    %plot(JERR); 
                    
                    if Cmax > .5 
                        error('Position correction is diverging.'); 
                        %return;  % Continue simulation, without correction.  
                    end
                    %disp(['  Joint ' num2str(J.jointID) ' position error (correction '  num2str(PiterCount) '): ' num2str(norm(C_t1))]);
                end
                totalPerr = totalPerr + norm(C_t1); 
          end
      end

      if Pcorrected
          %% Update Jacobian values between position and velocity correction
          for j=1:nj
             J = sim.joints(j);
             J.update();          
             clear Gb_t0 MinvGb0 J.bender.Gb_t0 J.bender.MinvGb0 C_t1; 
             [Gb_t1, ~, ~] = J.getJointCorrectionValues();  
             J.bender.Gb_t1 = Gb_t1; 
             J.bender.MinvGb_t1 = J.getMassInertiaMatrix() \ Gb_t1; 
          end


          %% Velocity correction
          ViterCount = 0;
          dCmax = inf; 
          Viter = 0; 
          while Viter < maxIters && dCmax > epsVelocity
              Viter = Viter+1;
              dCmax = 0;
              for j=1:nj
                    J = sim.joints(j); 
                    J.update();  % TODO: I think this is not necessary now, since only velocity is changed

                    b1 = J.body1;
                    b2 = J.body2; 

                    Gb_t1 = J.bender.Gb_t1;
                    Minv = J.bender.Minv; 
                    MinvGb1 = J.bender.MinvGb_t1; 

                    [~, ~, Cdot] = J.getJointCorrectionValues();
                    if norm(Cdot) > dCmax, dCmax = norm(Cdot); end

                    vi = 0;
                    while norm(Cdot) > epsVelocity && vi < 20;
                        ViterCount = ViterCount + 1;
                        vi = vi+1; 

                        dp = -inv(Gb_t1' * MinvGb1) * Cdot; 
                        dnu = MinvGb1 * dp;                         

                        % Update velocities
                        if b1.static
                            b2.nu = b2.nu + dnu(1:6); 
                        elseif b2.static
                            b1.nu = b1.nu + dnu(1:6);
                        else
                            b1.nu = b1.nu + dnu(1:6);
                            b2.nu = b2.nu + dnu(7:12); 
                        end

                        J.update(); 
                        [~, ~, Cdot] = J.getJointCorrectionValues(); 
                        if norm(Cdot) > dCmax, dCmax = norm(Cdot); end
                        
                        %disp(['     ' num2str(vi) ': ' num2str(norm(Cdot))]);

                        %disp(['     Joint ' num2str(J.jointID) ' velocity error (correction '  num2str(PiterCount) '): ' num2str(norm(Cdot))]);
                    end
              end
          end
      end
      %disp(['Piters (' num2str(Cmax) '): ' num2str(PiterCount) ',  Viters (' num2str(dCmax) '): ' num2str(ViterCount)]);
  
    end

end

