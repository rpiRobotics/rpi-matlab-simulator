function  mNCPdynamics( obj )
%MNCPDYNAMICS Summary of this function goes here

%Detailed explanation goes here
%NCPDYNAMICS formulate the friction cone as a quadratic friction cone,
%without any polyhedral facets to approximate or linearize the model.
%
% Nonlinear and quardratic
M = obj.dynamics.M;
Gn = obj.dynamics.Gn;
Gf = obj.dynamics.Gf;
U = obj.dynamics.U;
NU = obj.dynamics.NU;
FX = obj.dynamics.FX;
PSI = obj.dynamics.PSI;
nc = length(obj.Contacts); 
h = obj.h;
MinvPext =  M \ (FX*h);
end