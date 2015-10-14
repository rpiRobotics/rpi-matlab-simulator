// Takes a quaternion q and a vector r, and returns the result of rotating
// r with q.  The result is formatted as a column vector (3x1).  
// MATLAB Example: 
//      q = [0.995 0.0267 0.08 0.0534];
//      r1 = [1 2 3];
//      r2 = Qrotate(q,r1); 

#include "mex.h"

// Calculate the rotation of a vector by a quaternion q.  
void qrotate(double q[], double r[], double qout[])
{
    double a = q[0]; 
    double b = q[1];
    double c = q[2];
    double d = q[3]; 
    double as = a*a;
    double bs = b*b;
    double cs = c*c;
    double ds = d*d; 
    // This is what I think the formulation should be
    //qout[0] = (as+bs-cs-ds)*r[0] + (2*b*c-2*a*d)*r[1] + (2*b*d+2*a*c)*r[2];
    //qout[1] = (2*b*c+2*a*d)*r[0] + (as-bs+cs-ds)*r[1] + (2*c*d-2*a*b)*r[2];
    //qout[2] = (2*b*d-2*a*c)*r[0] + (2*c*d+2*a*b)*r[1] + (as-bs-cs+ds)*r[2];
    // This is what MATLAB wants it to be
    qout[0] = (as+bs-cs-ds)*r[0] + (2*b*c+2*a*d)*r[1] + (2*b*d-2*a*c)*r[2];
    qout[1] = (2*b*c-2*a*d)*r[0] + (as-bs+cs-ds)*r[1] + (2*c*d+2*a*b)*r[2];
    qout[2] = (2*b*d+2*a*c)*r[0] + (2*c*d-2*a*b)*r[1] + (as-bs-cs+ds)*r[2];
    // I haven't checked, but I  believe the difference manifests as a 
    // handed-ness difference.  
}

// Entry point for MATLAB call to this code.  
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  double *q,*r,*qout;  

  /* Create matrix for the return argument. */
  plhs[0] = mxCreateDoubleMatrix(3,1, mxREAL);
  
  /* Assign pointers to each input and output. */
  q = mxGetPr(prhs[0]);  
  r = mxGetPr(prhs[1]);  
  qout = mxGetPr(plhs[0]);  
  
  /* Call the cross subroutine. */
  qrotate(q,r,qout);
}



