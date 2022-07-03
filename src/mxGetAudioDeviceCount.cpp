/*
 * mxGetAudioDeviceCount.cpp (version 1.1.0)
 *
 * Returns the number of audio devices found by the
 * system that support the specified number of input,
 * output or duplex channels for the specified API string
 * name.
 *
 * The calling syntax is:
 *
 *		nDevices = mxGetAudioDeviceCount( nOutputs, nInputs, nDuplex, api )
 *
 * The first input argument is required, while the others are optional. Unspecified
 * values of nInputs or nDuplex are set to zero. If the API is not specified, the
 * default compiled API will be used.
 *
 * This is a MEX file for MATLAB by Gary Scavone, McGill University, 2018-2022.
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
  
  if ( nrhs > 4 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:nrhs",
                       "Too many input arguments." );
  }
  
  // Make sure first 3 input arguments are scalars
  int nArgs = (nrhs==4) ? 3 : nrhs;
  for ( int n=0; n<nArgs; n++ ) {
    if ( !mxIsDouble( prhs[n] ) ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:notScalar",
                         "The first three input arguments must be scalars." );
    }
  }

  // If the API is specified, make sure it is a string.
  if ( nrhs == 4 && ( !mxIsChar( prhs[3] ) || mxGetM( prhs[3]) != 1 ) ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:notString",
                       "The API input argument must be a string." );
  }
  
  unsigned int nOutputs = 0, nInputs = 0, nDuplex = 0;
  RtAudio::Api apiVal = RtAudio::UNSPECIFIED;
    
  nOutputs = mxGetScalar( prhs[0] );
  if ( nrhs > 1 ) nInputs = mxGetScalar( prhs[1] );
  if ( nrhs > 2 ) nDuplex = mxGetScalar( prhs[2] );
  if ( nrhs > 3 ) {
    char api[20];
    int status = mxGetString( prhs[3], api, sizeof(api) );
    if ( status != 0 )
      mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:mxGetString",
                         "Failed to copy input string into memory." );
    apiVal = RtAudio::getCompiledApiByDisplayName( std::string( api ) );
    if ( apiVal == RtAudio::UNSPECIFIED )
      mexErrMsgIdAndTxt( "mxGetAudioDeviceCount:apiNotFound",
                         "The specified API is not found." );
  }

  RtAudio audio( apiVal );
  audio.showWarnings( false );
  RtAudio::DeviceInfo info;
  std::vector<unsigned int> ids = audio.getDeviceIds();
  unsigned int count = 0;

  for ( unsigned int n=0; n<ids.size(); n++ ) {
    info = audio.getDeviceInfo( ids[n] );
    if ( info.outputChannels >= nOutputs && info.inputChannels >= nInputs &&
         info.duplexChannels >= nDuplex )
      count++;
  }
  
  // create the output variable
  double *y;
  plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL );
  y = mxGetPr(plhs[0]);
  *y = count;
  
  return;
}
