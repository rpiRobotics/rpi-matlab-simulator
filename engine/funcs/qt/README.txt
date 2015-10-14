
"qt" is a small quaternion library for MATLAB.  Quaternions are assumed to 
be represented as 4x1 column vectors where q = [a b c d]' and 'a' is the 
real component i.e. q = a + b*i + c*j + d*k.  

MATLAB does have quaternion functions in the Aerospace toolbox, but this 
toolbox may not be available to the user.  Also these functions give the 
quaternion in from body frame to world frame, which is left-handed or 
opposite of what we prefer to use with rigid body simulation.  Also, MATLAB 
stores treats quaternions as row vectors, and we use column vectors.  

 --- INCLUDED FUNCTIONS IN qt ---
   q = qt( k, theta )       : Returns a quaternion q corresponding to the rotation by theta radians about vector k.

   R = qt2rot( q )          : Returns a 3x3 rotation matrix R corresponding to the quaternion q.

qout = qtconj( q )          : Returns the quaternion conjugate qout of q.

qout = qtinv( q )           : Returns the quaternion inverse qout of q. 

vout = qtrotate( q, vin )   : Returns the rotated vector vout, rotated by quaternion q.

qout = qtmultiply( q1, q2 ) : Retruns the quaternion product of quaternions q1 and q2 (non-commutative).  
