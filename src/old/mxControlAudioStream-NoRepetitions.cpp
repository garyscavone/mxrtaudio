/*
 * controlAudioStream.cpp
 *
 * Open/start/stop/close an audio input/output stream.
 *
 * GENERAL USAGE:
 * 'Open' a stream with specified parameters, including a duration parameter
 * that sets the maximum time an input stream will run. Once 'started', the
 * stream will either stop itself when the maximum time is reached or it can
 * be stopped earlier with a 'stop' call. If a stream is stopped and then
 * restarted, previously recorded data in the internal buffer will be
 * overwritten.
 *
 * The calling syntax is:
 *
 *    result = controlAudioStream( 'open', iDuration, sampleRate,
 *                                  oChannels, iChannels, oSignal, 
 *                                  oDevice, iDevice,
 *                                  oChannelOffset, iChannelOffset )
 *
 *      with parameters:
 *      - sampleRate: audio sampling frequency (Hz)
 *      - oChannels: number of output channels
 *      - iChannels: number of input channels
 *      - oSignal: output signal vector for output (one column per channel)
 *      - oDevice: optional output device index (default = 0)
 *      - iDevice: optional input device index (default = 0)
 *      - oChannelOffset / iChannelOffset: optional output / input channel
 *            offsets on audio device (default = 0)
 *
 * For audio output only, 'iChannels' should be set to zero and the stream
 * will run for the duration of the 'oSignal' parameter ('iDuration' and
 * other input parameters will be ignored). For audio input only, 'oChannels'
 * should be set to zero and the other output parameters will be ignored.
 *
 *    OR
 *
 *    result = controlAudioStream( 'getData', mxArray )
 *
 *      where:
 *      - mxArray is a Matlab array to be filled with input data (one column
 *            per channel, the number of rows corresponds to the number of
 *            sample frames to be filled)
 *           
 * Recorded data can be retrieved from the internal buffer using the 'getData'
 * call. The function will wait until sufficient data has been recorded to
 * fill the mxArray input argument. If the mxArray size is larger than the
 * internal buffer, the function will block until the stream has stopped.
 * Consecutive calls with an mxArray size that is smaller than the internal
 * buffer will return sequential, adjacent blocks of data from the internal
 * buffer.
 *
 *    OR
 *
 *    result = controlAudioStream( 'start/stop/close' )
 *
 * A previously opened stream should be 'closed' to properly release the
 * RtAudio instance and free internal memory.
 *
 * This is a MEX file for MATLAB, created by Gary P. Scavone, McGill University, 2018.
*/

#include "mex.h"
#include <cmath>
#include "RtAudio.h"

// Platform-dependent sleep routines.
#if defined( __WINDOWS_ASIO__ ) || defined( __WINDOWS_DS__ ) || defined( __WINDOWS_WASAPI__ )
  #include <windows.h>
  #define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds ) 
#else // Unix variants
  #include <unistd.h>
  #define SLEEP( milliseconds ) usleep( (unsigned long) (milliseconds * 1000.0) )
#endif

struct TickData {
  unsigned int oChannels;
  unsigned int iChannels;
  double * source;
  double * response;
  unsigned long sourceFrames;
  unsigned long responseFrames;
  unsigned int frameCounter;
  unsigned int tickCounter;
  unsigned int readCounter;
  unsigned int sampleRate;

  TickData()
    : oChannels(0), iChannels(0), source(0), response(0), sourceFrames(0),
            responseFrames(0), frameCounter(0), tickCounter(0),
            readCounter(0), sampleRate(0) {}
};

// Global variables
RtAudio *audio = 0;
TickData tickData;

void cleanup( void )
{
  if ( audio ) {
    delete audio;
    audio = 0;
  }

  if ( tickData.source ) {
    delete [] tickData.source;
    tickData.source = 0;
  }
  
  if ( tickData.response ) {
    delete [] tickData.response;
    tickData.response = 0;
  }

  mexUnlock();
}

int tick( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
          double streamTime, RtAudioStreamStatus status, void *userData )
{  
  TickData *data = &tickData;
  if ( data->tickCounter++ < 10 ) { // skip first few ticks
    if ( data->oChannels ) {
      memset( outputBuffer, 0, nBufferFrames * data->oChannels * sizeof( double ) );
    }
    return 0;
  }
  int nFrames, frameCount = data->frameCounter;
  data->frameCounter += nBufferFrames;
  
  if ( data->oChannels ) { // audio output
    nFrames = data->sourceFrames - frameCount;
    if ( nFrames > 0 ) { // source frames to copy
      if ( nFrames > nBufferFrames ) nFrames = nBufferFrames;
      memcpy( outputBuffer, &data->source[frameCount*data->oChannels],
              nFrames * data->oChannels * sizeof( double ) );
      int framesLeft = nBufferFrames - nFrames;
      if ( framesLeft ) { // zero out remaining part of buffer
        double *oBuffer = (double *) outputBuffer;
        memset( &oBuffer[nFrames*data->oChannels], 0,
                framesLeft * data->oChannels * sizeof( double ) );
        if ( data->iChannels == 0 ) return 1;
      }
    }
    else { // no source frames to copy
      double *oBuffer = (double *) outputBuffer;
      memset( oBuffer, 0, nBufferFrames * data->oChannels * sizeof( double ) );
      if ( data->iChannels == 0 ) return 1;
    }
  }
  
  if ( data->iChannels ) { // audio input
    double *iBuffer = (double *) inputBuffer;
    nFrames = data->responseFrames - frameCount;
    if ( nFrames > 0 ) { // response frames to fill
      if ( nFrames > nBufferFrames ) nFrames = nBufferFrames;
      memcpy( &data->response[frameCount*data->iChannels], iBuffer,
              nFrames * data->iChannels * sizeof( double ) );
    }
    else return 1; // input buffer size will always be >= output buffer size
  }
  
  return 0;
}

// gateway function
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("controlAudioStream: First input should be a command string less than 64 characters long.");

  // Stop stream (but don't close it)
  if ( !strcmp( "stop", cmd )) {
    if ( audio )
      if ( audio->isStreamRunning() )
        audio->stopStream();
      else
        mexWarnMsgTxt("controlAudioStream: the stream is not running!");
    else
      mexWarnMsgTxt("controlAudioStream: no stream is open!");
    return;
  }
  
  // Close stream
  if ( !strcmp( "close", cmd )) {
    if ( audio ) {
      audio->closeStream();
      cleanup();
    }
    else
      mexWarnMsgTxt("controlAudioStream: no stream is open!");
    return;
  }

  // Start stream
  if ( !strcmp( "start", cmd )) {
    if ( audio ) {
      tickData.frameCounter = 0;
      tickData.readCounter = 0;
      tickData.tickCounter = 0;
      audio->startStream();
    }
    else
      mexWarnMsgTxt("controlAudioStream: no stream is open!");
    return;
  }

  // Return a block of input data
  if ( !strcmp( "getData", cmd )) {
    if ( nrhs != 2 ) {
      mexErrMsgIdAndTxt( "controlAudioStream:getData:nrhs",
              "controlAudioStream::getData: Two inputs required." );
      // Exit implicit
    }
    if ( !audio ) {
      mexWarnMsgTxt("controlAudioStream::getData: no stream is open!");
      return;
    }
    if ( tickData.iChannels == 0 ) {
      mexWarnMsgTxt("controlAudioStream::getData: the stream is output only!");
      return;
    }

    // If current data read pointer is at end of buffer, return and warn
    if ( tickData.readCounter >= tickData.responseFrames ) {
      mexWarnMsgTxt("controlAudioStream::getData: all data returned!");
      return;
    }

    mxArray *mxData = (mxArray *)prhs[1];
    double *data = (double *) mxGetPr(mxData);
    size_t N = mxGetN( mxData ); // channels
    size_t M = mxGetM( mxData ); // frames
    if ( N != tickData.iChannels ) {
      mexWarnMsgTxt("controlAudioStream::getData: number of channels in matrix argument not equal to number of input channels!");
      return;
    }

    int nFrames = M;
    // Check that we stay within the size of our response data
    if ( tickData.readCounter + M > tickData.responseFrames )
      nFrames = tickData.responseFrames - tickData.readCounter;
    
    // If fill mark is not beyond where updated read pointer will be, then wait.
    while ( tickData.readCounter + M > tickData.frameCounter ) {
      if ( audio->isStreamRunning() ) {
        double waitTime = 1000.0 * (tickData.readCounter + M - tickData.frameCounter ) / tickData.sampleRate;
        SLEEP( (unsigned long) waitTime );
      }
      else {
       nFrames = tickData.frameCounter - tickData.readCounter;
       if ( nFrames > 0 ) break;
       mexWarnMsgTxt("controlAudioStream::getData: stream is stopped and no data left to read!");
       return;
      }
    }
    
    // For consistency, return data de-interleaved, one channel per column.
    double *iBuffer = &tickData.response[tickData.readCounter*tickData.iChannels];
    for ( int n=0; n<N; n++ ) { // channels
      for ( int m=0; m<nFrames; m++ ) { // frames ... could be less than M
        *data++ = iBuffer[n+m*N]; // deinterleave data
        //data[m+n*M] = iBuffer[n+m*N]; // deinterleave data
      }
    }
    //memcpy( data, &(tickData.response[tickData.readCounter*tickData.iChannels]),
    //        M * tickData.iChannels * sizeof(double) );

    tickData.readCounter += nFrames;
    return;
  }
  
  // Incorrect argument
  if ( strcmp( "open", cmd )) {
    mexErrMsgIdAndTxt("controlAudioStream:open",
            "controlAudioStream::open: invalid first string argument!");
    return;
  }

  // Open stream
  if ( audio ) {
    mexWarnMsgTxt("controlAudioStream: a stream is already open!");
    return;
  }

  // Check number of input arguments
  if ( nrhs < 5 ) {
    mexErrMsgIdAndTxt( "controlAudioStream:open:nrhs",
            "controlAudioStream::open: At least four input arguments required." );
  }

  if ( nrhs > 10 ) {
    mexErrMsgIdAndTxt( "controlAudioStream:open:nrhs",
            "controlAudioStream::open: Too many input arguments." );
  }
  
// Make sure all input arguments are scalars (other than 'oSignal')
  for ( int n=1; n<nrhs; n++ ) {
    if ( n == 5 ) continue;
    if ( !mxIsDouble( prhs[n] ) ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "controlAudioStream:open:notScalar",
              "controlAudioStream:open: Input arguments (except oSignal) must be scalars.");
    }
  }

//    result = controlAudioStream( 'open', iDuration, sampleRate,
//                                  oChannels, iChannels, oSignal, 
//                                  oDevice, iDevice,
//                                  oChannelOffset, iChannelOffset )
  
  double iDuration = mxGetScalar( prhs[1] );
  int iChannels = mxGetScalar( prhs[4] );
  if ( iChannels > 0 && iDuration <= 0.1 ) {
    mexWarnMsgTxt("controlAudioStream::open: iDuration parameter is invalid!");
    return;
  }
  
  int sampleRate = mxGetScalar( prhs[2] );
  int oChannels = mxGetScalar( prhs[3] );
  int oDevice = 0;
  int iDevice = 0;
  int oChannelOffset = 0;
  int iChannelOffset = 0;
  if ( nrhs > 6 ) oDevice = mxGetScalar( prhs[6] );
  if ( nrhs > 7 ) iDevice = mxGetScalar( prhs[7] );
  if ( nrhs > 8 ) oChannelOffset = mxGetScalar( prhs[8] );
  if ( nrhs > 9 ) iChannelOffset = mxGetScalar( prhs[9] );

  tickData.oChannels = oChannels;
  if ( nrhs > 5 && oChannels > 0) {
    mxArray *mxData = (mxArray *)prhs[5];
    double *data = (double *) mxGetPr(mxData);
    size_t N = mxGetN( mxData ); // columns
    size_t M = mxGetM( mxData ); // rows
    if ( N != oChannels ) {
      mexWarnMsgTxt("controlAudioStream::open: number of channels in source signal not equal to 'oChannels' argument!");
      return;
    }
    
    // The Matlab array data is stored by column, which corresponds to
    // non-interleaved buffers. However, we can more easily block copy
    // buffers in the callback function as interleaved data.
    tickData.sourceFrames = M;
    tickData.source = new double[tickData.sourceFrames * oChannels];
    for ( int n=0; n<N; n++ ) { // channels
      for ( int m=0; m<M; m++ ) // frames
        tickData.source[n+m*N] = *data++; // interleave data
    }
    //memcpy( tickData.source, data, tickData.sourceFrames * oChannels * sizeof( double ) );
  }

  // Set the stream information
  RtAudio::StreamParameters iParams, oParams;
  iParams.deviceId = iDevice;
  iParams.nChannels = iChannels;
  oParams.deviceId = oDevice;
  oParams.nChannels = oChannels;
  oParams.firstChannel = oChannelOffset;
  iParams.firstChannel = iChannelOffset;

  RtAudio::StreamOptions options;
  //options.flags = RTAUDIO_NONINTERLEAVED;
  //unsigned int bufferFrames = 1024;
  unsigned int bufferFrames = 64;
  
  mexLock();
  audio = new RtAudio();
  
  // We'll let RtAudio do the parameter checking and print warnings.
  audio->showWarnings( true );
  
  // Try to open the realtime hardware
  try {
    if ( oChannels == 0 ) // input only
      audio->openStream( NULL, &iParams, RTAUDIO_FLOAT64,
              sampleRate, &bufferFrames, &tick, (void *)NULL, &options );
    else if ( iChannels == 0 ) // output only
      audio->openStream( &oParams, NULL, RTAUDIO_FLOAT64,
              sampleRate, &bufferFrames, &tick, (void *)NULL, &options );
    else // duplex
      audio->openStream( &oParams, &iParams, RTAUDIO_FLOAT64,
              sampleRate, &bufferFrames, &tick, (void *)NULL, &options );
  }
  catch ( RtAudioError& e ) {
    std::cout << e.getMessage() << std::endl;
    cleanup();
    mexErrMsgIdAndTxt( "controlAudioStream:open", e.getMessage().c_str());
  }
  
  tickData.sampleRate = sampleRate;
  tickData.iChannels = iChannels;
  if ( iChannels > 0 ) {
    tickData.responseFrames = ceil( iDuration * sampleRate );
    if ( tickData.sourceFrames > tickData.responseFrames )
      tickData.responseFrames = tickData.sourceFrames;
    if ( tickData.response == 0 )
      tickData.response = new double[tickData.responseFrames * iChannels];
    memset( tickData.response, 0, tickData.responseFrames*iChannels*sizeof(double));
  }
  
  return;
}