
/* 
 * RPI-MATLAB-Simulator
 * http://code.google.com/p/rpi-matlab-simulator/
 * June 14, 2012
 * 
 * This assumes the input arguments are two 3x1 vectors, 
 * and returns their cross product.  
 *
 */

#include "mex.h"

/* Calculate cross product of two vectors.  C = A x B. */
void cross(double A[], double B[], double C[])
{
  C[0] = A[1]*B[2] - A[2]*B[1]; 
  C[1] = A[2]*B[0] - A[0]*B[2];
  C[2] = A[0]*B[1] - A[1]*B[0]; 
}

/* Entry point for MATLAB call to this code.  */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  double *A,*B,*C;

  /* Create matrix for the return argument. */
  plhs[0] = mxCreateDoubleMatrix(3,1, mxREAL);
  
  /* Assign pointers to each input and output. */
  A = mxGetPr(prhs[0]);  
  B = mxGetPr(prhs[1]);  
  C = mxGetPr(plhs[0]);  
  
  /* Call the cross subroutine. */
  cross(A,B,C);
}
