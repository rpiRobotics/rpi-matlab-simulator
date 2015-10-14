%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_cube.m 
%
% Generates a rectangular mesh object 
% Inputs: width, length, height
%       
%         width
%      *----------*
%      |\          \
%      | \          \ length
%      *  \          \
%   ^   \  *----------*
%    \   \ |          |
%   y \   \|          | height 
%      \   *----------*
%       '---------->
%             x

function block = mesh_rectangularBlock( width, length, height )

    % We'll just make a cube then move the vertices 
    block = mesh_read_poly_file('cube.poly'); 
    
    % Width
    block.verts_local([1 2 3 4],1) = -width/2;
    block.verts_local([5 6 7 8],1) =  width/2;
    
    % Length
    block.verts_local([1 2 5 6],2) = -length/2;
    block.verts_local([3 4 7 8],2) =  length/2;
    
    % Height
    block.verts_local([1 3 5 7],3) = -height/2;
    block.verts_local([2 4 6 8],3) =  height/2; 
    
    block.J = diag((1/12)*block.mass*[length^2+height^2 width^2+height^2 width^2+length^2]);
    
    %block.Jinv = inv(cube.J); 
end

