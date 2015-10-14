

% Compile, including the necessary bullet libraries and include directory

cd engine/collision_detection/BULLET


if isunix
    if exist('/usr/local/include/bullet','dir')
        mex BULLET.cpp -I/usr/local/include/bullet -I/usr/local/include/bullet/BulletDynamics -I/usr/local/include/bullet/BulletCollision -I/usr/local/include/bullet/LinearMath -L/usr/local/lib -lBulletCollision -lBulletDynamics -lLinearMath 
    else
        disp('ERROR: Did not find bullet installation at /usr/local/include/bullet');
    end
elseif ismac
    % TODO
elseif ispc
    % TODO
else
    disp('ERROR: unkown architecture');
end


cd ../../../
