

function sim = wam_controller( sim )

    %% The joint indexes are as follows:
    %  1 : Base -> Shoulder1
    %  2 : Shoulder1 -> Shoulder2
    %  3 : Shoulder2 -> Arm
    %  4 : Arm -> Elbow
    %  7 : Wrist1 -> Wrist2 / Palm
    %  9 : Palm -> F1a
    % 10 : Fa1 -> F1b
    % 11 : F1b -> F1d
    % 12 : Palm -> F2a
    % 13 : F2a -> F2b
    % 14 : F2b -> F2d
    % 17 : F3a -> F3b
    % 18 : F3b -> F3d
    
    %% Init
    if sim.step == 1
       sim.userData.errorPrev = zeros(13,1);  
    end

    %% Controller
    %target_joint_angles = zeros(14,1);
    %% Approach object
    t_1 = 3;  % Time when approach is reached
    t_2 = 8;  % Time when grasp is completed
    %if sim.time <= t_1
        target_joint_angles_t1 = [ 0
                                      pi/4
                                      0 
                                      pi/2
                                      pi/4
                                      pi/6     % Palm -> F1a
                                      0     % F1a -> F1b
                                      0     % F1b -> F1d
                                      -pi/6     % Palm -> F2a
                                      0     % F2a -> F2b
                                      0     % F2b -> F2d
                                      0     % F3a -> F3b
                                      0 ];  % F3b -> F3d

        % Cubic polynomial interpolation
        t = min([sim.time t_1])/t_1;  
        target_joint_angles = target_joint_angles_t1*(-2*t^3 + 3*t^2);
        %target_joint_angles = (min([sim.time 2])/2) * target_joint_angles_FINAL;
    
    %% Grasp
    %else
    if sim.time > t_1
        2;
    end
%     if sim.time > t_1
%         target_joint_angles_t2 = [ 0
%                                       pi/4
%                                       0 
%                                       pi/2
%                                       pi/4
%                                       pi/6     % Palm -> F1a
%                                       pi/2     % F1a -> F1b
%                                       pi/2     % F1b -> F1d
%                                       -pi/6     % Palm -> F2a
%                                       pi/2     % F2a -> F2b
%                                       pi/2     % F2b -> F2d
%                                       pi/2     % F3a -> F3b
%                                       pi/2 ];  % F3b -> F3d
% 
%         % Cubic polynomial interpolation
%         t = min([(sim.time-t_1) (t_2-t_1)])/(t_2-t_1);  
%         target_joint_angles = target_joint_angles_t1 + (target_joint_angles_t2-target_joint_angles_t1)*(-2*t^3 + 3*t^2);
%         %target_joint_angles = (min([sim.time 2])/2) * target_joint_angles_FINAL;
%     %end
%     end
    
    
    % Clear body torques
    for j=1:length(sim.joints)
       sim.bodies(sim.joints(j).body1id).Fext(4:6) = 0; 
       sim.bodies(sim.joints(j).body2id).Fext(4:6) = 0; 
    end
    
    % Gains
    P = [ 1000
          6000 
          1000
          3500  % Elbow
          3500  % Wrist
          150
          80
          80
          150
          80
          80
          150
          80 ];
    D = .1*P; 
%     D = [ 1.5 
%           300
%           1.5
%           75
%           15
%           2
%           8
%           5
%           2
%           8
%           5
%           8
%           5 ]; 
    
    % Calculate new torques
    Joint_Index = [1 2 3 4 7 9 10 11 12 13 14 16 17];
                  
    for j=1:13
       J = sim.joints(Joint_Index(j));
       joint_error = target_joint_angles(j)-J.theta;
       d_joint_error = (joint_error-sim.userData.errorPrev(j))/sim.h;  
       sim.joints(j).theta_prev = J.theta; 
       sim.userData.errorPrev(j) = joint_error; 
       sim.userData.joint_error(sim.step,j) = joint_error; 
       
       joint_frame_torque = P(j)*joint_error + D(j)*d_joint_error;  

       t1 = J.T1world(1:3,1:3) * [0;0; -joint_frame_torque];  
       sim.bodies(J.body1id).Fext(4:6) = sim.bodies(J.body1id).Fext(4:6) + t1;

       t2 = J.T2world(1:3,1:3) * [0;0; joint_frame_torque];
       sim.bodies(J.body2id).Fext(4:6) = sim.bodies(J.body2id).Fext(4:6) + t2; 
    end
    
    % Plot joint information
    if sim.step == 1
       % WAM joints (first 5 joints)
       sim.userData.wam_joint_handles_target = gobjects(5); 
       sim.userData.wam_joint_handles_actual = gobjects(5); 
       sim.userData.wam_joint_fig = figure; 
       for i=1:5
          subplot(3,2,i);
          hold on; grid on;
          title(['Joint ' num2str(i)]);
          sim.userData.wam_joint_handles_target(i) = plot(sim.step,target_joint_angles(i));
          sim.userData.wam_joint_handles_actual(i) = plot(sim.step,sim.joints(Joint_Index(i)).theta,'r');
       end
           
       % Hand joints (remaining 8 joints)
       sim.userData.hand_joint_handles_target = gobjects(8); 
       sim.userData.hand_joint_handles_actual = gobjects(8); 
       sim.userData.hand_joint_fig = figure; 
       for i=6:13
          subplot(3,3,i-5);
          hold on; grid on;
          title(['Joint ' num2str(i-5)]);
          sim.userData.hand_joint_handles_target(i-5) = plot(sim.step,target_joint_angles(i));
          sim.userData.hand_joint_handles_actual(i-5) = plot(sim.step,sim.joints(Joint_Index(i)).theta,'r');
       end
    % Not first time step
    else
        for i=1:5
          set(sim.userData.wam_joint_handles_target(i),'XData',[get(sim.userData.wam_joint_handles_target(i),'XData') sim.step]);
          set(sim.userData.wam_joint_handles_target(i),'YData',[get(sim.userData.wam_joint_handles_target(i),'YData') target_joint_angles(i)]);
          set(sim.userData.wam_joint_handles_actual(i),'XData',[get(sim.userData.wam_joint_handles_actual(i),'XData') sim.step]);
          set(sim.userData.wam_joint_handles_actual(i),'YData',[get(sim.userData.wam_joint_handles_actual(i),'YData') sim.joints(Joint_Index(i)).theta]);
       end
       for i=6:13
          set(sim.userData.hand_joint_handles_target(i-5),'XData',[get(sim.userData.hand_joint_handles_target(i-5),'XData') sim.step]);
          set(sim.userData.hand_joint_handles_target(i-5),'YData',[get(sim.userData.hand_joint_handles_target(i-5),'YData') target_joint_angles(i)]);
          set(sim.userData.hand_joint_handles_actual(i-5),'XData',[get(sim.userData.hand_joint_handles_actual(i-5),'XData') sim.step]);
          set(sim.userData.hand_joint_handles_actual(i-5),'YData',[get(sim.userData.hand_joint_handles_actual(i-5),'YData') sim.joints(Joint_Index(i)).theta]);
       end
    end
    
    
end

