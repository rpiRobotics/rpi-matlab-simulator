
RPI-MATLAB-Simulator 
http://code.google.com/p/rpi-matlab-simulator/
README_MEX.txt

 - June 14, 2012 
     This folder contains pairs of .mex files and their MATLAB .m equivalents.  
     Some functions (cross product for example) are called A LOT during simulation.  Therefore, we include C code in order to increase the efficiency of the simulator.  However, many MATLAB installs have mex compiling issues, so we also include .m versions of these functions to fall back on.  We have attempted to use function names that will not conflict with built-in MATLAB functions.  

HOW IT WORKS
Setup.m will attempt to compile these .c functions.  
     If this is successful, MATLAB *should* call the compiled mex function with precedence (according to http://www.mathworks.com/help/techdoc/matlab_prog/f7-58170.html).  This can be confirmed after compiling with the MATLAB 'which' function.
     If this was not successful, then MATLAB will simply call the available .m functions.  