/*
 * mxGetAudioApis.cpp (version 1.0.0)
 *
 * Returns the names of the available, compiled audio apis for which
 * at least one device supports the specified number of input, output
 * and/or duplex channels.
 *
 * The calling syntax is:
 *
 *		audioApis = mxGetAudioApis( nOutputs, nInputs, nDuplex )
 *
 * The input arguments are optional. If none are provided, all compiled
 * apis will be returned (even if no devices are found for an api).
 *
 * This is a MEX file for MATLAB by Gary Scavone, McGill University, 2022.
*/

#include "mex.h"
#include "RtAudio.h"

// gateway function
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if ( nrhs > 3 ) {
    mexErrMsgIdAndTxt( "mxGetAudioDeviceName:nrhs",
                       "Too many input arguments." );
  }
  
  // Make sure all input arguments are scalars
  for ( int n=0; n<nrhs; n++ ) {
    if ( !mxIsDouble( prhs[n] ) ||
         mxIsComplex(prhs[n]) ||
         mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxGetAudioDeviceName:notScalar",
                         "All input arguments must be scalars.");
    }
  }
  
  unsigned int nOutputs = 0, nInputs = 0, nDuplex = 0;
  if ( nrhs > 0 ) nOutputs = mxGetScalar( prhs[0] );
  if ( nrhs > 1 ) nInputs = mxGetScalar( prhs[1] );
  if ( nrhs > 2 ) nDuplex = mxGetScalar( prhs[2] );

  std::vector<RtAudio::Api> apis, apis2return;
  RtAudio::getCompiledApi( apis );
  RtAudio *audio;

  if ( nOutputs > 0 || nInputs > 0 || nDuplex > 0 ) {
    for ( unsigned int n=0; n<apis.size(); n++ ) {
      audio = new RtAudio( apis[n] );
      audio->showWarnings( false );
      std::vector<unsigned int> ids = audio->getDeviceIds();
      for ( unsigned int m=0; m<ids.size(); m++ ) {
        RtAudio::DeviceInfo info = audio->getDeviceInfo( ids[m] );
        if ( info.outputChannels >= nOutputs && info.inputChannels >= nInputs &&
             info.duplexChannels >= nDuplex ) {
          // We have at least one device meeting the channel requirements, so return this API name
          apis2return.push_back( apis[n] );
          break;
        }
      }
      delete audio;
    }
  }
  else
    apis2return = apis;    

  mxArray *cell_array_ptr;
  cell_array_ptr = mxCreateCellMatrix( 1, apis2return.size() );
  for ( unsigned int n=0; n<apis2return.size(); n++ ) {
    std::string name = RtAudio::getApiDisplayName( apis2return[n] );
    mxArray *fout = mxCreateString( name.c_str() );
    mxSetCell( cell_array_ptr, n, fout );
  }

  plhs[0] = cell_array_ptr;
  return;
}
