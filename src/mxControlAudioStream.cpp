/*
 * mxControlAudioStream.cpp (version 1.1.0)
 *
 * Start/stop/restart/getData/close an audio input/output stream.
 *
 * GENERAL USAGE:
 *
 * 'Start' a stream with specified parameters. Depending on the parameters,
 * the stream will either stop itself or run continuously until manually
 * stopped and/or closed. A stopped stream can be restarted. Input data can
 * be retrieved with 'getData' calls. The stream must be 'closed' to
 * properly release the RtAudio instance and free internal memory. An
 * output signal can be repeated a specified number of times. Input data
 * acquisition can be delayed until 'triggered'. Non-zero function return
 * values generally indicate improper usage cases. Improper function syntax
 * or critical hardware problems result in thrown errors.
 *
 * STARTING A STREAM:
 *
 * For output-only streams (oChannels > 0, iChannels = 0), the stream will
 * stop itself after oSignal duration times nRepetitions+1. For input-only
 * streams (oChannels = 0, iChannels > 0) and iDuration > 0, the stream
 * will stop itself after iDuration seconds. For input and output streams
 * (oChannels > 0 and iChannels > 0) with iDuration > 0, the stream will
 * stop itself after the greater of iDuration or oSignal duration times
 * nRepetitions+1.
 * 
 * If iChannels > 0 and iDuration < 0, the stream will run continuously
 * until manually stopped or closed.
 *
 * The calling syntax is:
 *
 *    result = mxControlAudioStream( 'start', oChannels, iChannels, sampleRate,
 *                                   iDuration, oSignal, nRepetitions,
 *                                   oDeviceId, iDeviceId, triggerThreshold,
 *                                   triggerChannel, triggerTimeout,
 *                                   triggerBackup, oChannelOffset,
 *                                   iChannelOffset, nBufferFrames, api )
 *
 *    with parameters:
 *      - oChannels: number of output channels
 *      - iChannels: number of input channels
 *      - sampleRate: audio sampling frequency (Hz)
 *      - iDuration: input stream time (seconds, negative for continuous input)
 *      - oSignal: output signal vector for output (one row per channel)
 *      - nRepetitions: number of times to repeat oSignal (default = 0)
 *      - oDeviceId: optional output device ID (default = 0)
 *      - iDeviceId: optional input device ID (default = 0)
 *      - triggerThreshold: signal value at which a trigger occurs (default = 0)
 *      - triggerChannel: input channel in which to look for trigger 
 *        (default = 0 = first input)
 *      - triggerTimeout: maximum time (seconds) to wait for trigger (default = 5)
 *      - triggerBackup: number of samples to backup from trigger position
 *           before recording (default = 0)
 *      - oChannelOffset / iChannelOffset: optional output / input channel
 *           offsets on audio device (default = 0)
 *      - nBufferFrames: # of samples per frame for I/O (default = 512)
 *      - api: audio API string name (default = first compiled API found)
 *
 * Calls to this function should be wrapped in a try/catch block (all
 * syntax, parameter or hardware problems are thrown).
 *
 * For audio output only, iChannels should be set to zero and the stream
 * will run for the duration of the oSignal times nRepetitions+1 
 * (iDuration and other input parameters will be ignored). For audio input
 * only, oChannels should be set to zero and the other output parameters
 * will be ignored (use [] for oSignal if specifying more parameters).
 *
 * For audio input and output and iDuration > 0, the internal input buffer
 * will be resized (if necessary) to be as long as the oSignal duration
 * times nRepetitions+1. The iDuration can be longer than the oSignal
 * duration times nRepetitions.
 *
 * Triggering will only occur if triggerThreshold > 0.0. The triggerBackup
 * value cannot be larger than nBufferFrames and must be positive. 
 *
 * RETRIEVING INPUT DATA:
 *
 *    result = mxControlAudioStream( 'getData', mxArray )
 *
 *      where:
 *      - mxArray is a Matlab array to be filled with input data (one row
 *           per channel, the number of columns corresponds to the number
 *           of sample frames to be filled)
 *
 * Input data can be retrieved from the internal buffer using the 'getData'
 * call. The function will block until sufficient data has been recorded to
 * fill the mxArray input argument. If the mxArray size is equal to or
 * larger than the internal buffer (when iDuration > 0), the function will
 * block until the stream has stopped. When iDuration < 0 (continuous input),
 * the mxArray size must be less than or equal to 0.5 seconds of sample 
 * frames. Consecutive calls with an mxArray size that is smaller than the
 * internal buffer will return sequential, adjacent blocks of data from the
 * internal buffer. Note that this call will block if an input stream is
 * waiting for a trigger.
 *
 * Improper syntax (lack of an mxArray) will produce a thrown error. All
 * other possible problems are reported with a non-zero return value and a
 * warning message (such as a stream stopping without sufficient
 * accumulated data to fill the mxArray).
 *
 * STOPPING A STREAM:
 *
 *    result = mxControlAudioStream( 'stop' )
 *
 * A previously opened stream can be 'stopped' at any time. A non-zero
 * return value indicates a warning condition (such as an attempt to stop
 * a non-existent or already stopped stream).
 *
 * RESTARTING A STREAM:
 *
 *    result = mxControlAudioStream( 'restart' )
 *
 * A previously opened and then stopped stream can be 'restarted.' It will
 * use all previous stream settings and run as when first started (output
 * will start from the beginning of oSignal; any previously acquired input
 * data will be discarded). A non-zero return value indicates a warning
 * condition (such as an attempt to restart a non-existent or running
 * stream).
 *
 * CLOSING A STREAM:
 *
 *    result = mxControlAudioStream( 'close', force )
 *
 * A previously opened stream should be 'closed' to properly release the
 * RtAudio instance and free internal memory. The 'force' parameter is
 * optional. When 'force' = true, the stream will be stopped and closed
 * immediately. Otherwise, the function will wait until the stream is
 * finished before closing. However, if the stream is "continuous"
 * (iDuration < 0), no 'force' parameter is necessary.
 *
 * This is a MEX file for MATLAB, created by Gary P. Scavone,
 * McGill University, 2018-2020.
*/

#include "mex.h"
#include <cmath>
#include <cstring>
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
  double * lastInputBuffer;
  unsigned long sourceFrames;
  unsigned long responseFrames;
  unsigned int sFrameCounter;
  unsigned int rFrameCounter;
  unsigned long tickCounter;
  unsigned int repeatCounter;
  unsigned int readCounter;
  unsigned int sampleRate;
  unsigned int nRepetitions;
  unsigned int triggerChannel;
  double triggerThreshold;
  unsigned int triggerTimeout;
  unsigned int triggerBackup;
  unsigned int triggerCounter;
  bool triggerFlag; // 1 when waiting for trigger, 0 otherwise
  bool wrapFlag;
  bool continuous;
  
  TickData()
    : oChannels(0), iChannels(0), source(0), response(0), lastInputBuffer(0),
            sourceFrames(0), responseFrames(0), sFrameCounter(0),
            rFrameCounter(0), tickCounter(0), repeatCounter(0), readCounter(0),
            sampleRate(0), nRepetitions(0), triggerChannel(0),
            triggerThreshold(0.0), triggerTimeout(0), triggerBackup(0),
            triggerCounter(0), triggerFlag(0), wrapFlag(0), continuous(0) {}
};

// Global variables
RtAudio *audio = 0;
TickData *tickData = 0;

void reset( void ) // clear counters and flags for restart
{
  if ( !tickData ) return;
  tickData->sFrameCounter = 0;
  tickData->rFrameCounter = 0;
  tickData->tickCounter = 0;
  tickData->repeatCounter = 0;
  tickData->readCounter = 0;
  tickData->triggerCounter = 0;
  if ( tickData->triggerThreshold > 0.0 )
    tickData->triggerFlag = true;
  else
    tickData->triggerFlag = false;
  tickData->wrapFlag = false;
}

void cleanup( void )
{
  if ( audio ) {
    delete audio;
    audio = 0;
  }
  
  if ( tickData ) {
    if ( tickData->response )
      delete [] tickData->response;
    if ( tickData->lastInputBuffer )
      delete [] tickData->lastInputBuffer;
    delete tickData;
    tickData = 0;
  }

  mexUnlock();
}

// The RtAudio callback function.
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
    if ( data->triggerFlag ) {
      double *iBuffer = (double *) inputBuffer;
      // Look for a trigger
      for ( unsigned int n=0; n<nBufferFrames; n++ ) {
        if ( std::abs(iBuffer[n*data->iChannels + data->triggerChannel]) >= data->triggerThreshold ) {
          // Have a trigger ... check backup
          int iStart = n - data->triggerBackup;
          if ( iStart < 0 ) {
            // First copy from the previous buffer
            nFrames = -iStart;
            iStart = nBufferFrames + iStart;
            memcpy( data->response, &data->lastInputBuffer[iStart*data->iChannels], 
                    nFrames * data->iChannels * sizeof( double ) );
            data->rFrameCounter += nFrames;
            iStart = 0;
          }
          // Copy from the current buffer
          nFrames = nBufferFrames - iStart;
          memcpy( &data->response[data->rFrameCounter*data->iChannels],
                  &iBuffer[iStart*data->iChannels],
                  nFrames * data->iChannels * sizeof( double ) );
          data->rFrameCounter += nFrames;
          data->triggerFlag = false;
          return 0;
        }
      }
      // No trigger ... check timeout
      data->triggerCounter += nBufferFrames;
      if ( data->triggerCounter >= data->triggerTimeout )
        return 2; // timeout
      // Save this buffer for possible backup
      memcpy( data->lastInputBuffer, inputBuffer,
              nBufferFrames * data->iChannels * sizeof( double ) );
      return 0;
    }
    frameCount = data->rFrameCounter;
    nFrames = data->responseFrames - frameCount;
    if ( nFrames > 0 ) { // response frames to fill
      if ( nFrames > nBufferFrames ) nFrames = nBufferFrames;
      memcpy( &data->response[frameCount*data->iChannels], inputBuffer,
              nFrames * data->iChannels * sizeof( double ) );
      data->rFrameCounter += nFrames;
    }
  
    if ( data->continuous && data->rFrameCounter == data->responseFrames ) {
      data->rFrameCounter = 0;
      data->wrapFlag = true;
      if ( nFrames < nBufferFrames ) { // finish emptying the input buffer
        double *iBuffer = (double *) inputBuffer;
        iBuffer += nFrames * data->iChannels;
        int framesLeft = nBufferFrames - nFrames;
        memcpy( &data->response[0], iBuffer, framesLeft * data->iChannels * sizeof( double ) );
        data->rFrameCounter = framesLeft;
      }
      return 0;
    }
    if ( data->rFrameCounter >= data->responseFrames ) return 1;
  }
  
  return 0;
}

// The MEX gateway function.
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  plhs[0] = mxCreateDoubleScalar( 1 ); // start with default return value of one for errors
  double *retval = mxGetPr(plhs[0]);
  *retval = 1; // initially set to indicate a warning or error

  char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("mxControlAudioStream: First input should be a command string less than 64 characters long.");

  // Stop a stream immediately
  if ( !strcmp( "stop", cmd )) {
    if ( audio ) {
      if ( audio->isStreamRunning() )
        audio->stopStream();
      *retval = 0;
    }
    else
      mexWarnMsgTxt("mxControlAudioStream::stop: no stream is open!");
    return;
  }

  // Stop and close stream
  if ( !strcmp( "close", cmd )) {
    if ( audio ) {
      if ( audio->isStreamRunning() ) {
        if ( !tickData->continuous && (nrhs < 2 || ( nrhs > 1 && mxGetScalar( prhs[1] ) != true) ) ) {
          while ( audio->isStreamRunning() ) { // wait until stopped
            SLEEP( 50 );
          }
        }
      }
      audio->closeStream();
      cleanup();
      *retval = 0;
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
    if ( !tickData->continuous && tickData->readCounter >= tickData->responseFrames ) {
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

    // If continuous operation, restrict return vector size to 0.5 seconds
    if ( tickData->continuous ) {
      if ( N > ceil(0.5 * tickData->sampleRate) ) {
        mexWarnMsgTxt("mxControlAudioStream::getData: number of frames in matrix argument must be less than 0.5 seconds of data!");
        return;
      }
    }
    
    int nFrames = N;
    // Check that we stay within the size of our response data buffer
    if ( tickData->readCounter + N > tickData->responseFrames )
      nFrames = tickData->responseFrames - tickData->readCounter;
    
    if ( tickData->wrapFlag == false ) {
      // If fill mark is not beyond where updated read pointer will be, then wait.
      int waitFrames = tickData->readCounter + nFrames - tickData->rFrameCounter;
      //std::cout << "initial waitFrames = " << waitFrames << ", isRunning = " << audio->isStreamRunning() << std::endl;
      while ( waitFrames > 0 ) {
        if ( audio->isStreamRunning() ) {
          //std::cout << "before wait" << std::endl;
          double waitTime = 1100.0 * waitFrames / tickData->sampleRate;
          SLEEP( (unsigned long) waitTime );
          //std::cout << "waited for " << waitTime << " ms, count = " << tickData->rFrameCounter << std::endl;
          if ( tickData->wrapFlag ) break;
          waitFrames = tickData->readCounter + nFrames - tickData->rFrameCounter;
          //std::cout << ", new waitFrames = " << waitFrames << ", isRunning = " << audio->isStreamRunning() << ", count = " << tickData->rFrameCounter << std::endl;
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
    }
    
    // Return interleaved data, one channel per row.
    memcpy( data, &tickData->response[tickData->readCounter*tickData->iChannels],
            nFrames * tickData->iChannels * sizeof( double ) );
    //std::cout << "readCounter = " << tickData->readCounter << ", nFrames = " << nFrames << ", rFrameCounter = " << tickData->rFrameCounter << ", responseFrames = " << tickData->responseFrames << std::endl;
    tickData->readCounter += nFrames;
    
    if ( tickData->wrapFlag && tickData->readCounter == tickData->responseFrames ) {
      tickData->wrapFlag = false;
      tickData->readCounter = 0;
      //std::cout << "in getData, wrapFlag, nFrames = " << nFrames << ", N = " << N << std::endl;
      if ( nFrames < N ) { // there are frames left to fill from beginning of buffer after wrap around
        int framesLeft = N - nFrames;
        int waitFrames = framesLeft - tickData->rFrameCounter;
        //std::cout << "in getData, framesLeft = " << framesLeft << ", waitFrames = " << waitFrames << std::endl;
        while ( waitFrames > 0 ) {
          double waitTime = 1100.0 * waitFrames / tickData->sampleRate;
          SLEEP( (unsigned long) waitTime );
          if ( tickData->wrapFlag ) break;
          waitFrames = framesLeft - tickData->rFrameCounter;
        }
        //std::cout << "in getData, after waiting" << std::endl;
        memcpy( &data[nFrames*tickData->iChannels], &tickData->response[0],
                framesLeft*tickData->iChannels * sizeof( double ) );
        tickData->readCounter = framesLeft;
      }
    }
    *retval = 0;
    return;
  }

  // Restart a stream
  RtAudioErrorType result;
  if ( !strcmp( "restart", cmd )) {
    if ( !audio ) {
      mexWarnMsgTxt("mxControlAudioStream::restart: no stream is open!");
      return;
    }
    if ( audio->isStreamRunning() ) {
      mexWarnMsgTxt("mxControlAudioStream::restart: stream is still running!");
      return;
    }
    reset();
    result = audio->startStream();
    if ( result ) {
      std::string errorText = audio->getErrorText();
      cleanup();
      mexErrMsgIdAndTxt( "mxControlAudioStream:restart", errorText.c_str());
    }
    *retval = 0;
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

  if ( nrhs > 17 ) {
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:nrhs",
            "mxControlAudioStream::start: Too many input arguments." );
  }
  
// Make sure all input arguments are scalars (other than 'oSignal', 'iDuration' and 'api')
  for ( int n=1; n<nrhs; n++ ) {
    if ( n == 4 ) continue;
    if ( n == 5 ) continue;
    if ( n == 16 ) continue;
    if ( !mxIsDouble( prhs[n] ) ||
            mxGetScalar( prhs[n] ) < 0 ||
            mxIsComplex(prhs[n]) ||
            mxGetNumberOfElements(prhs[n]) != 1 ) {
      *retval = 1;
      mexErrMsgIdAndTxt( "mxControlAudioStream:start:notScalar",
              "mxControlAudioStream:start: Input arguments (except iDuration, oSignal and api) must be positive scalars.");
    }
  }

//    result = mxControlAudioStream( 'start', oChannels, iChannels, sampleRate,
//                                    iDuration, oSignal, nRepetitions,
//                                    oDeviceId, iDeviceId, triggerThreshold,
//                                    triggerChannel, triggerTimeout,
//                                    triggerBackup, oChannelOffset,
//                                    iChannelOffset, nBufferFrames, api )


  int oChannels = mxGetScalar( prhs[1] );
  int iChannels = mxGetScalar( prhs[2] );
  int sampleRate = mxGetScalar( prhs[3] );
  double iDuration = mxGetScalar( prhs[4] );
  int nRepetitions = 0;
  int oDeviceId = 0;
  int iDeviceId = 0;
  double triggerThreshold = 0.0;
  int triggerChannel = 0;
  int triggerTimeout = 5;
  int triggerBackup = 0;
  int oChannelOffset = 0;
  int iChannelOffset = 0;
  unsigned int bufferFrames = 512;
  if ( nrhs > 6 && oChannels ) nRepetitions = mxGetScalar( prhs[6] );
  if ( nrhs > 7 ) oDeviceId = mxGetScalar( prhs[7] );
  if ( nrhs > 8 ) iDeviceId = mxGetScalar( prhs[8] );
  if ( nrhs > 9 ) triggerThreshold = mxGetScalar( prhs[9] );
  if ( nrhs > 10 ) triggerChannel = mxGetScalar( prhs[10] );
  if ( nrhs > 11 ) triggerTimeout = mxGetScalar( prhs[11] );
  if ( nrhs > 12 ) triggerBackup = mxGetScalar( prhs[12] );
  if ( nrhs > 13 ) oChannelOffset = mxGetScalar( prhs[13] );
  if ( nrhs > 14 ) iChannelOffset = mxGetScalar( prhs[14] );
  if ( nrhs > 15 ) bufferFrames = mxGetScalar( prhs[15] );
  // If the API is specified, make sure it is a string.
  if ( nrhs > 16 && ( !mxIsChar( prhs[16] ) || mxGetM( prhs[16]) != 1 ) )
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:apiNotString",
                       "The API input argument must be a string." );
  if ( iChannels > 0 && triggerChannel >= iChannels )
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:triggerChannel",
            "mxControlAudioStream::start: trigger channel is greater than number of input channels!");
  if ( triggerTimeout < 0 )
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:triggerTimeout",
            "mxControlAudioStream::start: trigger timeout value is less than zero!");
  if ( triggerBackup < 0 )
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:triggerBackup",
            "mxControlAudioStream::start: trigger backup is less than zero!");
  if ( triggerBackup > bufferFrames )
    mexErrMsgIdAndTxt( "mxControlAudioStream:start:triggerBackup",
            "mxControlAudioStream::start: trigger backup must be less <= bufferFrames." );

  RtAudio::Api apiVal = RtAudio::UNSPECIFIED;
  if ( nrhs > 16 ) {
    char api[20];
    int status = mxGetString( prhs[16], api, sizeof(api) );
    if ( status != 0 )
      mexErrMsgIdAndTxt( "mxControlAudioStream:start:mxGetString",
                         "Failed to copy input string into memory." );
    apiVal = RtAudio::getCompiledApiByDisplayName( std::string( api ) );
    if ( apiVal == RtAudio::UNSPECIFIED )
      mexErrMsgIdAndTxt( "mxControlAudioStream:start:apiNotFound",
                         "The specified API is not found." );
  }
  
  double *data = 0;
  unsigned long oFrames = 0;
  if ( nrhs > 5 && oChannels > 0) {
    mxArray *mxData = (mxArray *)prhs[5];
    data = (double *) mxGetPr(mxData);
    oFrames = mxGetN( mxData ); // columns (frames)
    size_t M = mxGetM( mxData ); // rows (channels)
    if ( M != oChannels )
      mexErrMsgIdAndTxt( "mxControlAudioStream:start:sourceChannels",
              "mxControlAudioStream::start: number of channels in source signal not equal to 'oChannels' argument!");
    
    // The Matlab array data is stored sequentially by column, so if
    // channels were in columns, the buffers would correspond to
    // non-interleaved buffers. It is easier to work with interleaved data,
    // so we are going to force the user to provide arrays with channels in
    // rows so that we can simply block copy buffers as interleaved (or not
    // have to reformat the data). We will use an interleaved format with
    // RtAudio.
  }

  // Set the stream information
  RtAudio::StreamParameters iParams, oParams;
  iParams.deviceId = iDeviceId;
  iParams.nChannels = iChannels;
  oParams.deviceId = oDeviceId;
  oParams.nChannels = oChannels;
  oParams.firstChannel = oChannelOffset;
  iParams.firstChannel = iChannelOffset;

  mexLock();
  audio = new RtAudio( apiVal );
  
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
  if ( triggerThreshold > 0.0 ) {
    tickData->triggerFlag = true;
    tickData->triggerThreshold = triggerThreshold;
    tickData->triggerChannel = triggerChannel;
    tickData->triggerTimeout = (unsigned int) (triggerTimeout * sampleRate);
    tickData->triggerBackup = triggerBackup;
  }
  else
    tickData->triggerFlag = false;
  
  // Try to open the realtime hardware
  if ( oChannels == 0 ) // input only
    result = audio->openStream( NULL, &iParams, RTAUDIO_FLOAT64,
                                sampleRate, &bufferFrames, &tick, (void *)tickData );
  else if ( iChannels == 0 ) // output only
    result = audio->openStream( &oParams, NULL, RTAUDIO_FLOAT64,
                                sampleRate, &bufferFrames, &tick, (void *)tickData );
  else // duplex
    result = audio->openStream( &oParams, &iParams, RTAUDIO_FLOAT64,
                               sampleRate, &bufferFrames, &tick, (void *)tickData );
  if ( result > 0 ) {
    std::string errorText = audio->getErrorText();
    cleanup();
    mexErrMsgIdAndTxt( "mxControlAudioStream:start", errorText.c_str());
  }
  
  if ( iChannels > 0 ) {
    if ( iDuration < 0 ) {
      tickData->continuous = true;
      tickData->responseFrames = ceil( 2.0 * sampleRate );
    }
    else
      tickData->responseFrames = ceil( iDuration * sampleRate );
    int sFrames = tickData->sourceFrames * (1 + nRepetitions);
    if ( tickData->responseFrames < sFrames )
      tickData->responseFrames = sFrames;
    if ( tickData->response == 0 )
      tickData->response = new double[tickData->responseFrames * iChannels];
    memset( tickData->response, 0, tickData->responseFrames*iChannels*sizeof(double) );
    if ( tickData->triggerFlag && tickData->lastInputBuffer == 0 ) {
      tickData->lastInputBuffer = new double[bufferFrames * iChannels];
      memset( tickData->lastInputBuffer, 0, bufferFrames*iChannels*sizeof(double) );
    }
  }
  
  result = audio->startStream();
  if ( result ) {
    std::string errorText = audio->getErrorText();
    cleanup();
    mexErrMsgIdAndTxt( "mxControlAudioStream:start", errorText.c_str());
  }

  *retval = 0;
  return;
}
