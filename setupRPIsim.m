%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% This script sets up the path for RPIsim, so RUN ME!

fprintf('Setting up RPI-MATLAB-Simulator...  ');

addpath(genpath('engine'));  % Add every subfolder of 'engine'
addpath('engine');           % Add 'engine' itself

addpath(genpath('examples'));
addpath('examples'); 

fprintf(' finished setup. \n');
disp('Look in "examples" for example scripts. ');




