/*
 * mxGetAudioDeviceIds.cpp (version 1.0.0)
 *
 * Returns an array of audio device IDs found by the
 * system that support the specified number of input,
 * output or duplex channels for the specified API string
 * name.
 *
 * The calling syntax is:
 *
 *		ids = mxGetAudioDeviceIds( nOutputs, nInputs, nDuplex, api )
 *
 * The first input argument is required, while the others are optional. Unspecified
 * values of nInputs or nDuplex are set to zero. If the API is not specified, the
 * default compiled API will be used.
 *
 * This is a MEX file for MATLAB by Gary Scavone, McGill University, 2022.
*/

#include "mex.h"
#include "RtAudio.h"

// gateway function
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if ( nrhs < 1 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceIds:nrhs",
                       "At least one input required." );
  }
  
  if ( nrhs > 4 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceIds:nrhs",
                       "Too many input arguments." );
  }
  
  // Make sure first 3 input arguments are scalars
  int nArgs = (nrhs==4) ? 3 : nrhs;
  for ( int n=0; n<nArgs; n++ ) {
    if ( !mxIsDouble( prhs[n] ) ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxGetAudioDeviceIds:notScalar",
                         "The first three input arguments must be scalars." );
    }
  }

  // If the API is specified, make sure it is a string.
  if ( nrhs == 4 && ( !mxIsChar( prhs[3] ) || mxGetM( prhs[3]) != 1 ) ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceIds:notString",
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
      mexErrMsgIdAndTxt( "mxGetAudioDeviceIds:mxGetString",
                         "Failed to copy input string into memory." );
    apiVal = RtAudio::getCompiledApiByDisplayName( std::string( api ) );
    if ( apiVal == RtAudio::UNSPECIFIED )
      mexErrMsgIdAndTxt( "mxGetAudioDeviceIds:apiNotFound",
                         "The specified API is not found." );
  }

  RtAudio audio( apiVal );
  RtAudio::DeviceInfo info;
  std::vector<unsigned int> ids = audio.getDeviceIds();
  std::vector<unsigned int> returnIds;

  for ( unsigned int n=0; n<ids.size(); n++ ) {
    info = audio.getDeviceInfo( ids[n] );
    if ( info.outputChannels >= nOutputs && info.inputChannels >= nInputs &&
         info.duplexChannels >= nDuplex )
      returnIds.push_back( ids[n] );
  }
  
  // Create the output variable
  mwSize dims[] = {1, returnIds.size()};
  plhs[0] = mxCreateDoubleMatrix( 1, returnIds.size(), mxREAL );
  double *data = (double *)mxGetData(plhs[0]);
  for (unsigned int n=0; n<returnIds.size(); n++ )
    *data++ = (double) returnIds[n];
  
  return;
}
