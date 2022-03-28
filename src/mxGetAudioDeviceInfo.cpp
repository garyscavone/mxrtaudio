/*
 * mxGetAudioDeviceInfo.cpp (version 1.1.0)
 *
 * Returns an information structure for the specified audio device ID
 * and for the specified audio API string name.  The information
 * structure includes the device name, maximum numbers of output,
 * input and duplex channels, and the supported sample rates (the
 * rates are not comprehensive but rather selected from a list of
 * common audio sample rates). If no corresponding device is found,
 * the structure fields are empty.
 *
 * The calling syntax is:
 *
 *		deviceInfo = mxGetAudioDeviceInfo( deviceId, api )
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
    mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:nrhs",
                       "At least one input required." );
  }
  
  if ( nrhs > 2 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:nrhs",
                       "Too many input arguments." );
  }

  // Make sure the first input argument is a scalar
  if ( !mxIsDouble( prhs[0] ) || mxIsComplex(prhs[0]) ||
            mxGetNumberOfElements(prhs[0]) != 1 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:notScalar",
                       "The first input arguments must be a scalar.");
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
      mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:mxGetString",
                         "Failed to copy input string into memory." );
    apiVal = RtAudio::getCompiledApiByDisplayName( std::string( api ) );
    if ( apiVal == RtAudio::UNSPECIFIED )
      mexErrMsgIdAndTxt( "mxGetAudioDeviceInfo:apiNotFound",
                         "The specified API is not found." );
  }

  RtAudio audio ( apiVal );
  RtAudio::DeviceInfo info = audio.getDeviceInfo( id );
  
  // Create return structure
  const char *fieldnames[] = { "name", "outputChannels", "inputChannels",
                               "duplexChannels", "sampleRates" };
  plhs[0] = mxCreateStructMatrix(1, 1, 5, fieldnames);

  mxArray *fout = mxCreateString( info.name.c_str() );
  mxSetFieldByNumber(plhs[0], 0, 0, fout);
  fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
  double *y = mxGetPr(fout);
  *y = info.outputChannels;
  mxSetFieldByNumber(plhs[0], 0, 1, fout);
  fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
  y = mxGetPr(fout);
  *y = info.inputChannels;
  mxSetFieldByNumber(plhs[0], 0, 2, fout);
  fout = mxCreateDoubleMatrix( 1, 1, mxREAL );
  y = mxGetPr(fout);
  *y = info.duplexChannels;
  mxSetFieldByNumber(plhs[0], 0, 3, fout);
  fout = mxCreateDoubleMatrix( 1, info.sampleRates.size(), mxREAL );
  y = mxGetPr(fout);
  for ( unsigned int n=0; n<info.sampleRates.size(); n++ )
    y[n] = (double) info.sampleRates[n];
  mxSetFieldByNumber(plhs[0], 0, 4, fout);
  
  return;
}
