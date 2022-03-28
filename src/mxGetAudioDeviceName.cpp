/*
 * mxGetAudioDeviceName.cpp (version 1.1.0)
 *
 * Returns the name of the audio device specified by the
 * device ID and for the specified audio API string name.
 *
 * The calling syntax is:
 *
 *		deviceName = mxGetAudioDeviceName( deviceId, api )
 *
 * The deviceId input argument is required. If the API is not specified, the
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
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:nrhs",
            "At least one input required." );
  }
  
  if ( nrhs > 2 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:nrhs",
            "Too many input arguments." );
  }
  
  // Make sure the first input argument is a scalar
  if ( !mxIsDouble( prhs[0] ) || mxIsComplex(prhs[0]) ||
       mxGetNumberOfElements(prhs[0]) != 1 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:notScalar",
              "The first input argument must be a scalar.");
  }

  // If the API is specified, make sure it is a string.
  if ( nrhs == 2 && ( !mxIsChar( prhs[1] ) || mxGetM( prhs[1]) != 1 ) ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:notString",
                       "The API input argument must be a string." );
  }
  
  unsigned int id = mxGetScalar( prhs[0] );
  RtAudio::Api apiVal = RtAudio::UNSPECIFIED;
  if ( nrhs > 1 ) {
    char api[20];
    int status = mxGetString( prhs[1], api, sizeof(api) );
    if ( status != 0 )
      mexErrMsgIdAndTxt( "mxGetAudioDeviceName:mxGetString",
                         "Failed to copy input string into memory." );
    apiVal = RtAudio::getCompiledApiByDisplayName( std::string( api ) );
    if ( apiVal == RtAudio::UNSPECIFIED )
      mexErrMsgIdAndTxt( "mxGetAudioDeviceName:apiNotFound",
                         "The specified API is not found." );
  }

  RtAudio audio ( apiVal );
  RtAudio::DeviceInfo info = audio.getDeviceInfo( id );
  if ( info.name.empty() )
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:invalidId",
                       "The specified device ID is invalid." );
  
  plhs[0] = mxCreateString( info.name.c_str() );
  return;
}
