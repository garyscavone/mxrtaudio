/*
 * mxGetAudioDeviceInfo.cpp (version 1.0.0)
 *
 * Returns an information structure for the specified audio device index
 * (the first is iDevice = 1) that supports the specified number of input,
 * output or duplex channels. The information structure includes the device
 * name, maximum numbers of output, input and duplex channels, and the
 * supported sample rates (the rates are not comprehensive but rather
 * selected from a list of common audio sample rates). If no corresponding
 * device is found, the structure fields are empty.
 *
 * The calling syntax is:
 *
 *		deviceInfo = mxGetAudioDeviceInfo( iDevice, nOutputs, nInputs, nDuplex )
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
    mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:nrhs",
            "At least two inputs required." );
  }
  
  if ( nrhs > 4 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:nrhs",
            "Too many input arguments." );
  }
  
  // make sure all input arguments are scalars
  for ( int n=0; n<nrhs; n++ ) {
    if ( !mxIsDouble( prhs[n] ) ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:notScalar",
              "All input arguments must be scalars.");
    }
  }
  
  unsigned int iDevice, nOutputs = 0, nInputs = 0, nDuplex = 0;
  iDevice = mxGetScalar( prhs[0] );
  nOutputs = mxGetScalar( prhs[1] );
  if ( nrhs > 2 ) nInputs = mxGetScalar( prhs[2] );
  if ( nrhs > 3 ) nDuplex = mxGetScalar( prhs[3] );
  
  // Create return structure
  const char *fieldnames[] = {"name", "id", "outputChannels", "inputChannels",
                              "duplexChannels", "sampleRates" };
  plhs[0] = mxCreateStructMatrix(1, 1, 6, fieldnames);

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
      mxArray *fout = mxCreateString( info.name.c_str() );
      mxSetFieldByNumber(plhs[0], 0, 0, fout);
      fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
      double *y = mxGetPr(fout);
      *y = (double)n;
      mxSetFieldByNumber(plhs[0], 0, 1, fout);
      fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
      y = mxGetPr(fout);
      *y = info.outputChannels;
      mxSetFieldByNumber(plhs[0], 0, 2, fout);
      fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
      y = mxGetPr(fout);
      *y = info.inputChannels;
      mxSetFieldByNumber(plhs[0], 0, 3, fout);
      fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
      y = mxGetPr(fout);
      *y = info.duplexChannels;
      mxSetFieldByNumber(plhs[0], 0, 4, fout);
      fout = mxCreateDoubleMatrix( 1, info.sampleRates.size(), mxREAL );
      y = mxGetPr(fout);
      for ( unsigned int n=0; n<info.sampleRates.size(); n++ )
        y[n] = (double) info.sampleRates[n];
      mxSetFieldByNumber(plhs[0], 0, 5, fout);
      return;
    }
  }
  
  return;
}
