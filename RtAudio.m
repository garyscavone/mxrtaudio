classdef RtAudio < handle
% CLASS RtAudio
%
% This class provides a uniform interface within Matlab for audio IO
% support using RtAudio. RtAudio functionality is provided via underlying
% MEX functions.
%
% Gary Scavone, McGill University, 2018-2020.

  properties (SetObservable)
    isValid = false;
    streamIsOpen = 0;
    version = 1.0;
  end
  
  methods
  
    %-----------------------------------------------------------
    function obj = RtAudio()
      if exist( 'mxGetAudioDeviceCount', 'file' ) ...
          && exist( 'mxGetAudioDeviceName', 'file' ) ...
          && exist( 'mxGetAudioDeviceInfo', 'file' ) ...
          && exist( 'mxControlAudioStream', 'file' )
        obj.isValid = true;
      else
        error( 'RtAudio: Some or all mex functions not found!' );
      end
    end
    

    %-----------------------------------------------------------
    function delete( obj )
      if obj.streamIsOpen
        mxControlAudioStream( 'close', true );
      end
      clear mxControlAudioStream;
    end
      
      
    %-----------------------------------------------------------
    function list = listDevices( obj, show )
      if nargin < 2, show = false; end
      nInputs = obj.getAudioDeviceCount( 0, 1 );
      nOutputs = obj.getAudioDeviceCount( 1, 0 );
      list = '';
      for n = 1:nOutputs
        if n == 1
          list = sprintf( 'Output Devices: \n' );
        end
        name = sprintf( '\t%d: %s\n', n, obj.getAudioDeviceName( n, 1, 0 ) );
        list = [list name]; %#ok<*AGROW>
      end
      for n = 1:nInputs
        if n == 1
          list = [list sprintf( 'Input Devices: \n' ) ];
        end
        name = sprintf( '\t%d: %s\n', n, obj.getAudioDeviceName( n, 0, 1 ) );
        list = [list name];
      end
      if show
        fprintf( '%s', list );
      end
    end


    %-----------------------------------------------------------
    function count = getAudioDeviceCount(~, nOutputs, nInputs)
      count = 0;
      try
        count = mxGetAudioDeviceCount(nOutputs, nInputs, 0);
      catch
        error('RtAudio.m: error calling getAudioDeviceCount.');
      end
    end
    
    
    %-----------------------------------------------------------
    function name = getAudioDeviceName(~, iDevice, nOutputs, nInputs)
      name = '';
      try
        name = mxGetAudioDeviceName(iDevice, nOutputs, nInputs, 0);
      catch
        error('RtAudio.m: error calling getAudioDeviceName.');
      end
    end

    
    %-----------------------------------------------------------
    function info = getAudioDeviceInfo(~, iDevice, nOutputs, nInputs)
      info = struct( 'name', [], 'id', [], 'outputChannels', [], ...
        'inputChannels', [], 'duplexChannels', [], 'sampleRates', [] );
      try
        info = mxGetAudioDeviceInfo(iDevice, nOutputs, nInputs, 0);
      catch
        error('RtAudio.m: error calling getAudioDeviceInfo.');
      end
    end
    
    
    %-----------------------------------------------------------
    function startAudioStream(obj, oChannels, iChannels, sampleRate, ...
                              iDuration, oSignal, nRepetitions, ...
                              oDevice, iDevice, triggerThreshold, ...
                              triggerChannel, triggerTimeout, triggerBackup, ...
                              oChannelOffset, iChannelOffset, nBufferFrames)
      if obj.streamIsOpen
        warning('RtAudio::startAudioStream: a stream is already open!');
        return;
      end
      if nargin < 6, oSignal = []; end
      if nargin < 7, nRepetitions = 0; end
      if nargin < 8, oDevice = 0; end
      if nargin < 9, iDevice = 0; end
      if nargin < 10, triggerThreshold = 0; end
      if nargin < 11, triggerChannel = 0; end
      if nargin < 12, triggerTimeout = 0; end
      if nargin < 13, triggerBackup = 0; end
      if nargin < 14, oChannelOffset = 0; end
      if nargin < 15, iChannelOffset = 0; end
      if nargin < 16, nBufferFrames = 512; end

      [ochans, ~] = size( oSignal );
      if ochans ~= oChannels
        warning('RtAudio::startAudioStream: oSignal must have oChannels rows!');
        return;
      end
      
      try
        mxControlAudioStream( 'start', oChannels, iChannels, sampleRate, ...
          iDuration, oSignal, nRepetitions, ...
          oDevice, iDevice, triggerThreshold, ...
          triggerChannel, triggerTimeout, triggerBackup, ...
          oChannelOffset, iChannelOffset, nBufferFrames );
      catch ME
        error(['RtAudio.m: error calling startAudioStream. ', ME.message]);
      end
      obj.streamIsOpen = 1;
    end


    %-----------------------------------------------------------
    function closeAudioStream(obj, force)
      if obj.streamIsOpen
        if nargin < 2, force = 0; end
        mxControlAudioStream( 'close', force );
        obj.streamIsOpen = 0;
      else
        warning('RtAudio::closeAudioStream: no stream open!');
      end
    end

    
    %-----------------------------------------------------------
    function stopAudioStream(obj)
      if obj.streamIsOpen
        mxControlAudioStream( 'stop' );
      else
        warning('RtAudio::stopAudioStream: no stream open!');
      end
    end

    
    %-----------------------------------------------------------
    function restartAudioStream(obj)
      if obj.streamIsOpen
        mxControlAudioStream( 'restart' );
      else
        warning('RtAudio::restartAudioStream: no stream open!');
      end
    end

    
    %-----------------------------------------------------------
    function retval = getAudioData(obj, y)
      if obj.streamIsOpen
        retval = mxControlAudioStream( 'getData', y );
      else
        warning('RtAudio::getAudioData: no stream open!');
      end
    end

  end % methods
  
end