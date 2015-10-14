%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_replay.m
%
% Re-animates a simulation from data log files

% Input:
%   data_dir    - The directory containing the log files from a simulation.
%
% In order to generate a data_dir from a simulation, enable RECORD_DATA
% i.e. sim.RECORD_DATA = true; before calling sim.run; 
function sim_replay(data_dir)
  %clear all; 
  
  % Load the bodies from the simulation
  load([data_dir '/bodies.mat']);    
  num_bodies = length(bodies);  
  
  % Read the log file 
  sim_data = csvread([data_dir '/sim_log.txt']); % Read in data
  num_steps = size(sim_data,1); 
  disp(['Reading data from ' num2str(num_steps) ' time steps...']);
  
  % Initialize graphics handles for all objects
  figure(1); axis off; rotate3d; set(gcf,'Color',[1;1;1]);
  hold on; 
  for i=1:num_bodies
      bodies(i) = body_draw_init(bodies(i)); 
  end
  view(3); axis equal; xlabel('X'); ylabel('Y'); zlabel('Z'); 
  title(['Step: 1, Time: ' num2str(sim_data(1)) ' sec']); 
  
  % Slider
  plot_pos = get(gcf,'Position'); 
  hSlider = uicontrol('Style', 'slider',...
            'Min',1,'Max',num_steps,'Value',1,...
            'Position', [50 10 plot_pos(3)-100 20],...
            'Callback', {@slideTime,bodies,sim_data}); 
  %hProp = findprop(hSlider,'Value'); 
  %sliderListener = handle.listener(hSlider,hProp,'PropertyPostSet',{@playSim,bodies,sim_data,hSlider});
  %sliderListener = handle.addlistener(hSlider,'Value','PreSet',{@playSim,bodies,sim_data,hSlider}); 
        
  % Play button
  uicontrol('Style', 'pushbutton', 'String', 'Play',...
            'Position', [20 35 40 20],...
            'Callback', {@playSim,bodies,sim_data,hSlider});        
end

%%
function slideTime(hObj,event,bodies,sim_data)
  step = get(hObj,'Value'); 
  setTime(bodies,sim_data,step); 
end

%% 
function setTime(bodies,sim_data, step)
  step = int16(step);  % Assert that step is an int 
  num_bodies = length(bodies); 
  t = sim_data(step,1);
  for i=1:num_bodies
     bodies(i).u = sim_data(step,2+7*(i-1):1+7*i-4);
     bodies(i).quat = sim_data(step,1+7*(i-1)+4:1+7*i)';
     
     % Update union bodies
%      if strcmp(bodies(i).body_type, 'union')
%        for m = 1:bodies(i).num
%            bodies(i).UNION(m).quat = bodies(i).quat;       % Rotation 
%            bodies(i).UNION(m).u = bodies(i).u' + ...       % Translation
%                quatrotate(bodies(i).quat, bodies(i).UNION(m).offset')';
%        end
%      end
     
     body_draw(bodies(i)); 
  end
  drawnow; 
  title(['Step: ' num2str(step) ', Time: ' num2str(t) ' sec']); 
end

%% 
function playSim(hObj,event,bodies,sim_data,hSlider)
  for step = 1:size(sim_data,1)
     setTime(bodies,sim_data,step); 
     set(hSlider,'Value',step);
     %M(step) = getframe;
  end
  %save('movie.mat','M');
end












