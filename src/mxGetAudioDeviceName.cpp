/*
 * mxGetAudioDeviceName.cpp (version 1.0.0)
 *
 * Returns the name of the specified audio device index
 * that supports the specified number of input, output
 * or duplex channels.
 *
 * The calling syntax is:
 *
 *		deviceName = mxGetAudioDeviceName( iDevice, nOutputs, nInputs, nDuplex )
 *
 * This is a MEX file for MATLAB by Gary Scavone, McGill University, 2018.
*/

#include "mex.h"
#include "RtAudio.h"

// gateway function
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if ( nrhs < 2 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:nrhs",
            "At least two inputs required." );
  }
  
  if ( nrhs > 4 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:nrhs",
            "Too many input arguments." );
  }
  
  // make sure all input arguments are scalars
  for ( int n=0; n<nrhs; n++ ) {
    if ( !mxIsDouble( prhs[n] ) ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxGetAudioDeviceName:notScalar",
              "All input arguments must be scalars.");
    }
  }
  
  unsigned int iDevice, nOutputs = 0, nInputs = 0, nDuplex = 0;
  iDevice = mxGetScalar( prhs[0] );
  nOutputs = mxGetScalar( prhs[1] );
  if ( nrhs > 2 ) nInputs = mxGetScalar( prhs[2] );
  if ( nrhs > 3 ) nDuplex = mxGetScalar( prhs[3] );

  RtAudio audio;
  RtAudio::DeviceInfo info;
  unsigned int nDevices = audio.getDeviceCount();
  unsigned int count = 0;
  
  for ( unsigned int n=0; n<nDevices; n++ ) {
    info = audio.getDeviceInfo( n );
    if ( info.probed == true && info.outputChannels >= nOutputs &&
            info.inputChannels >= nInputs && info.duplexChannels >= nDuplex )
      count++;
    if ( count == iDevice ) {
      plhs[0] = mxCreateString( info.name.c_str() );
      return;
    }
  }
  
  plhs[0] = mxCreateString( NULL );
  return;
}
