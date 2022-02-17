/*
 * mxControlAudioStream.cpp
 *
 * Start/close an audio input/output stream.
 *
 * GENERAL USAGE:
 * 'Start' a stream with specified parameters. The stream will either stop
 * itself (for input only: after 'iDuration' seconds; for output only:
 * after 'oSignal' duration times nRepetitions; for input and output: after
 * the greater of 'iDuration' or 'oSignal' duration times nRepetitions) or
 * it can be closed earlier with a 'close' call.
 *
 * The calling syntax is:
 *
 *    result = mxControlAudioStream( 'start', oChannels, iChannels, sampleRate,
 *                                   iDuration, oSignal, nRepetitions,
 *                                   oDevice, iDevice, oChannelOffset,
 *                                   iChannelOffset, nBufferFrames )
 *
 *    with parameters:
 *      - oChannels: number of output channels
 *      - iChannels: number of input channels
 *      - sampleRate: audio sampling frequency (Hz)
 *      - iDuration: input stream time (seconds)
 *      - oSignal: output signal vector for output (one row per channel)
 *      - nRepetitions: number of times to repeat oSignal (default = 0)
 *      - oDevice: optional output device index (default = 0)
 *      - iDevice: optional input device index (default = 0)
 *      - oChannelOffset / iChannelOffset: optional output / input channel
 *           offsets on audio device (default = 0)
 *      - nBufferFrames: # of samples per frame for I/O
 *
 * For audio output only, 'iChannels' should be set to zero and the stream
 * will run for the duration of the 'oSignal' times nRepetitions 
 * ('iDuration' and other input parameters will be ignored). For audio
 * input only, 'oChannels' should be set to zero and the other output
 * parameters will be ignored (use [] for 'oSignal' if specifying more
 * parameters).
 *
 * For audio input and output, the internal input buffer will be resized
 * (if necessary) to be as long as the 'oSignal' duration times
 * nRepetitions. The 'iDuration' can be longer than the 'oSignal' duration
 * times nRepetitions.
 *
 *    OR
 *
 *    result = mxControlAudioStream( 'getData', mxArray )
 *
 *      where:
 *      - mxArray is a Matlab array to be filled with input data (one row
 *           per channel, the number of columns corresponds to the number
 *           of sample frames to be filled)
 *           
 * Recorded data can be retrieved from the internal buffer using the 'getData'
 * call. The function will block until sufficient data has been recorded to
 * fill the mxArray input argument. If the mxArray size is larger than the
 * internal buffer, the function will block until the stream has stopped.
 * Consecutive calls with an mxArray size that is smaller than the internal
 * buffer will return sequential, adjacent blocks of data from the internal
 * buffer.
 *
 *    OR
 *
 *    result = mxControlAudioStream( 'close', force )
 *
 * A previously opened stream should be 'closed' to properly release the
 * RtAudio instance and free internal memory. The 'force' parameter is
 * optional. When 'force' = true, the stream will be stopped and closed
 * immediately. Otherwise, the function will wait until the stream is
 * finished before closing.
 *
 * This is a MEX file for MATLAB, created by Gary P. Scavone,
 * McGill University, 2018-2019.
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
  unsigned int sFrameCounter;
  unsigned int rFrameCounter;
  unsigned long tickCounter;
  unsigned int repeatCounter;
  unsigned int readCounter;
  unsigned int sampleRate;
  unsigned int nRepetitions;
  
  TickData()
    : oChannels(0), iChannels(0), source(0), response(0), sourceFrames(0),
            responseFrames(0), sFrameCounter(0), rFrameCounter(0), 
            tickCounter(0), repeatCounter(0), readCounter(0),
            sampleRate(0), nRepetitions(0) {}
};

// Global variables
RtAudio *audio = 0;
TickData *tickData = 0;

void cleanup( void )
{
  if ( audio ) {
    delete audio;
    audio = 0;
  }
  
  if ( tickData ) {
    if ( tickData->response )
      delete [] tickData->response;
    delete tickData;
    tickData = 0;
  }

  mexUnlock();
}

int tick( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
          double streamTime, RtAudioStreamStatus status, void *userData )
{  
  TickData *data = (TickData *) userData;
  
  // Skip the first few calls to the tick() function as the initial data is
  // often noisy.
  if ( data->tickCounter++ < 5 ) {
    if ( data->oChannels ) {
      memset( outputBuffer, 0, nBufferFrames * data->oChannels * sizeof( double ) );
    }
    return 0;
  }

  int nFrames, frameCount;

  if ( data->oChannels ) { // audio output
    frameCount = data->sFrameCounter;
    data->sFrameCounter += nBufferFrames;
    nFrames = data->sourceFrames - frameCount;
    if ( nFrames > 0 ) { // source frames to copy
      if ( nFrames > nBufferFrames ) nFrames = nBufferFrames;
      memcpy( outputBuffer, &data->source[frameCount*data->oChannels],
              nFrames * data->oChannels * sizeof( double ) );
    }
    else nFrames = 0;
    int framesLeft = nBufferFrames - nFrames;
    if ( framesLeft ) {
      // cycle back if repeating, otherwise zero remainder of buffer
      double *oBuffer = (double *) outputBuffer;
      if ( data->repeatCounter++ < data->nRepetitions ) { // cycle back
        data->sFrameCounter = framesLeft;
        memcpy( &oBuffer[nFrames * data->oChannels], data->source,
                framesLeft * data->oChannels * sizeof( double ) );
      }
      else { // zero remainder of output buffer
        memset( &oBuffer[nFrames * data->oChannels], 0,
                framesLeft * data->oChannels * sizeof( double ) );
        if ( data->iChannels == 0 ) return 1;
      }
    }
  }
  
  if ( data->iChannels ) { // audio input
    frameCount = data->rFrameCounter;
    nFrames = data->responseFrames - frameCount;
    if ( nFrames > 0 ) { // response frames to fill
      if ( nFrames > nBufferFrames ) nFrames = nBufferFrames;
      memcpy( &data->response[frameCount*data->iChannels], inputBuffer,
              nFrames * data->iChannels * sizeof( double ) );
      data->rFrameCounter += nFrames;
    }
    if ( data->rFrameCounter >= data->responseFrames ) return 1;
  }
  
  return 0;
}

// gateway function
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("mxControlAudioStream: First input should be a command string less than 64 characters long.");

  // Stop and close stream
  if ( !strcmp( "close", cmd )) {
    if ( audio ) {
      if ( audio->isStreamRunning() ) {
        if ( nrhs < 2 || ( nrhs > 1 && mxGetScalar( prhs[1] ) != true ) ) {
          while ( audio->isStreamRunning() ) { // wait until stopped
            SLEEP( 50 );
          }
        }
      }
      audio->closeStream();
      cleanup();
    }
    else
      mexWarnMsgTxt("mxControlAudioStream::close: no stream is open!");
    return;
  }

  // Return a block of input data
  if ( !strcmp( "getData", cmd )) {
    if ( nrhs != 2 ) {
      mexErrMsgIdAndTxt( "mxControlAudioStream:getData:nrhs",
              "mxControlAudioStream::getData: Two inputs required." );
      // Exit implicit
    }
    if ( !audio ) {
      mexWarnMsgTxt("mxControlAudioStream::getData: no stream is open!");
      return;
    }
    if ( tickData->iChannels == 0 ) {
      mexWarnMsgTxt("mxControlAudioStream::getData: the stream is output only!");
      return;
    }

    // If current data read pointer is at end of buffer, return and warn
    if ( tickData->readCounter >= tickData->responseFrames ) {
      mexWarnMsgTxt("mxControlAudioStream::getData: all data returned!");
      return;
    }

    mxArray *mxData = (mxArray *)prhs[1];
    double *data = (double *) mxGetPr(mxData);
    size_t N = mxGetN( mxData ); // columns (frames)
    size_t M = mxGetM( mxData ); // rows (channels)
    if ( M != tickData->iChannels ) {
      mexWarnMsgTxt("mxControlAudioStream::getData: number of channels in matrix argument not equal to number of input channels!");
      return;
    }

    int nFrames = N;
    // Check that we stay within the size of our response data buffer
    if ( tickData->readCounter + N > tickData->responseFrames )
      nFrames = tickData->responseFrames - tickData->readCounter;
    
    // If fill mark is not beyond where updated read pointer will be, then wait.
    int waitFrames = tickData->readCounter + nFrames - tickData->rFrameCounter;
    //std::cout << "initial waitFrames = " << waitFrames << ", isRunning = " << audio->isStreamRunning() << std::endl;
    while ( waitFrames > 0 ) {
      if ( audio->isStreamRunning() ) {
        double waitTime = 1100.0 * waitFrames / tickData->sampleRate;
        SLEEP( (unsigned long) waitTime );
        //std::cout << "waited for " << waitTime << " ms, ";
        waitFrames = tickData->readCounter + nFrames - tickData->rFrameCounter;
        //std::cout << "new waitFrames = " << waitFrames << ", isRunning = " << audio->isStreamRunning() << std::endl;
      }
      else {
       nFrames = tickData->rFrameCounter - tickData->readCounter;
       if ( nFrames > N ) nFrames = N;
       //std::cout << "stream not running, nFrames = " << nFrames << std::endl;
       if ( nFrames > 0 ) break;
       mexWarnMsgTxt("mxControlAudioStream::getData: stream is stopped and no data left to read!");
       return;
      }
    }
    
    // Return interleaved data, one channel per row.
    memcpy( data, &tickData->response[tickData->readCounter*tickData->iChannels],
            nFrames * tickData->iChannels * sizeof( double ) );
    //std::cout << "readCounter = " << tickData->readCounter << ", nFrames = " << nFrames << ", rFrameCounter = " << tickData->rFrameCounter << ", responseFrames = " << tickData->responseFrames << std::endl;
    tickData->readCounter += nFrames;
    return;
  }
  
  // Incorrect argument
  if ( strcmp( "start", cmd )) {
    mexErrMsgIdAndTxt("mxControlAudioStream:start",
            "mxControlAudioStream: invalid first string argument!");
    return;
  }

  // Open and start stream
  if ( audio ) { // shouldn't be possible
    mexWarnMsgTxt("mxControlAudioStream::start: a stream is already open!");
    return;
  }

  // Check number of input arguments
  if ( nrhs < 5 ) {
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:nrhs",
            "mxControlAudioStream::start: At least four input arguments required." );
  }

  if ( nrhs > 12 ) {
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:nrhs",
            "mxControlAudioStream::start: Too many input arguments." );
  }
  
// Make sure all input arguments are scalars (other than 'oSignal')
  for ( int n=1; n<nrhs; n++ ) {
    if ( n == 5 ) continue;
    if ( !mxIsDouble( prhs[n] ) ||
            mxGetScalar( prhs[n] ) < 0 ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      mexErrMsgIdAndTxt( "mxControlAudioStream:start:notScalar",
              "mxControlAudioStream:start: Input arguments (except oSignal) must be positive scalars.");
    }
  }

//    result = mxControlAudioStream( 'start', oChannels, iChannels, sampleRate,
//                                  iDuration, oSignal, nRepetitions,
//                                  oDevice, iDevice, oChannelOffset, 
//                                  iChannelOffset, nBufferFrames )

  int oChannels = mxGetScalar( prhs[1] );
  int iChannels = mxGetScalar( prhs[2] );
  int sampleRate = mxGetScalar( prhs[3] );
  double iDuration = mxGetScalar( prhs[4] );
  int nRepetitions = 0;
  int oDevice = 0;
  int iDevice = 0;
  int oChannelOffset = 0;
  int iChannelOffset = 0;
  unsigned int bufferFrames = 512;
  if ( nrhs > 6 && oChannels ) nRepetitions = mxGetScalar( prhs[6] );
  if ( nrhs > 7 ) oDevice = mxGetScalar( prhs[7] );
  if ( nrhs > 8 ) iDevice = mxGetScalar( prhs[8] );
  if ( nrhs > 9 ) oChannelOffset = mxGetScalar( prhs[9] );
  if ( nrhs > 10 ) iChannelOffset = mxGetScalar( prhs[10] );
  if ( nrhs > 11 ) bufferFrames = mxGetScalar( prhs[11] );
  
  double *data = 0;
  unsigned long oFrames = 0;
  if ( nrhs > 5 && oChannels > 0) {
    mxArray *mxData = (mxArray *)prhs[5];
    data = (double *) mxGetPr(mxData);
    oFrames = mxGetN( mxData ); // columns (frames)
    size_t M = mxGetM( mxData ); // rows (channels)
    if ( M != oChannels ) {
      mexWarnMsgTxt("mxControlAudioStream::open: number of channels in source signal not equal to 'oChannels' argument!");
      return;
    }
    
    // The Matlab array data is stored by column, which corresponds to
    // non-interleaved buffers. If we force the user to provide buffers
    // with channels in rows, we can block copy buffers as interleaved.
    // Thus, we need to used an interleaved format with RtAudio.
  }

  // Set the stream information
  RtAudio::StreamParameters iParams, oParams;
  iParams.deviceId = iDevice;
  iParams.nChannels = iChannels;
  oParams.deviceId = oDevice;
  oParams.nChannels = oChannels;
  oParams.firstChannel = oChannelOffset;
  iParams.firstChannel = iChannelOffset;

  mexLock();
  audio = new RtAudio();
  
  // We'll let RtAudio do the parameter checking and print warnings.
  audio->showWarnings( true );
  
  if ( !tickData ) tickData = new TickData();
  if ( oChannels ) {
    tickData->source = data;
    tickData->sourceFrames = oFrames;
  }
  tickData->sampleRate = sampleRate;
  tickData->oChannels = oChannels;
  tickData->iChannels = iChannels;
  tickData->nRepetitions = nRepetitions;
  
  // Try to open the realtime hardware
  try {
    if ( oChannels == 0 ) // input only
      audio->openStream( NULL, &iParams, RTAUDIO_FLOAT64,
              sampleRate, &bufferFrames, &tick, (void *)tickData );
    else if ( iChannels == 0 ) // output only
      audio->openStream( &oParams, NULL, RTAUDIO_FLOAT64,
              sampleRate, &bufferFrames, &tick, (void *)tickData );
    else // duplex
      audio->openStream( &oParams, &iParams, RTAUDIO_FLOAT64,
              sampleRate, &bufferFrames, &tick, (void *)tickData );
  }
  catch ( RtAudioError& e ) {
    std::cout << e.getMessage() << std::endl;
    cleanup();
    mexErrMsgIdAndTxt( "mxControlAudioStream:start", e.getMessage().c_str());
  }
  
  if ( iChannels > 0 ) {
    tickData->responseFrames = ceil( iDuration * sampleRate );
    int sFrames = tickData->sourceFrames * (1 + nRepetitions);
    if ( tickData->responseFrames < sFrames )
      tickData->responseFrames = sFrames;
    if ( tickData->response == 0 )
      tickData->response = new double[tickData->responseFrames * iChannels];
    memset( tickData->response, 0, tickData->responseFrames*iChannels*sizeof(double) );
  }
  
  try {
    audio->startStream();
  }
  catch ( RtAudioError& e ) {
    std::cout << e.getMessage() << std::endl;
    cleanup();
    mexErrMsgIdAndTxt( "mxControlAudioStream:start", e.getMessage().c_str());
  }

  return;
}