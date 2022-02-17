/*
 * mxGetAudioDeviceCount.cpp (version 1.0.0)
 *
 * Returns the number of audio devices found by the
 * system that support the specified number of input,
 * output or duplex channels.
 *
 * The calling syntax is:
 *
 *		nDevices = mxGetAudioDeviceCount( nOutputs, nInputs, nDuplex )
 *
 * This is a MEX file for MATLAB by Gary Scavone, McGill University, 2018.
*/

#include "mex.h"
#include "RtAudio.h"

// gateway function
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if ( nrhs < 1 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:nrhs",
            "At least one input required." );
  }
  
  if ( nrhs > 3 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:nrhs",
            "Too many input arguments." );
  }
  
  // make sure all input arguments are scalars
  for ( int n=0; n<nrhs; n++ ) {
    if ( !mxIsDouble( prhs[n] ) ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:notScalar",
              "All input arguments must be scalars.");
    }
  }
  
  unsigned int nOutputs = 0, nInputs = 0, nDuplex = 0;
  nOutputs = mxGetScalar( prhs[0] );
  if ( nrhs > 1 ) nInputs = mxGetScalar( prhs[1] );
  if ( nrhs > 2 ) nDuplex = mxGetScalar( prhs[2] );

  RtAudio audio;
  RtAudio::DeviceInfo info;
  unsigned int nDevices = audio.getDeviceCount();
  unsigned int count = 0;

  for ( unsigned int n=0; n<nDevices; n++ ) {
    info = audio.getDeviceInfo( n );
    if ( info.probed == true && info.outputChannels >= nOutputs &&
            info.inputChannels >= nInputs && info.duplexChannels >= nDuplex )
      count++;
  }
  
  // create the output variable
  double *y;
  plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL );
  y = mxGetPr(plhs[0]);
  *y = count;
  
  return;
}
