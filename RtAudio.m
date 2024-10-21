classdef RtAudio < handle
% CLASS RtAudio
%
% This class provides a uniform interface within Matlab for audio IO
% support using RtAudio. RtAudio functionality is provided via underlying
% MEX functions.
%
% Gary Scavone, McGill University, 2018-2022.

  properties (SetObservable)
    isValid = false;
    streamIsOpen = 0;
    version = 1.1;
    api_ = '';
  end
  
  methods(Static)

    %-----------------------------------------------------------
    function apis = getAudioApis( nOutputs, nInputs )
      apis = '';
      if exist( 'mxGetAudioApis.mexmaci64', 'file' )
        apis = mxGetAudioApis( nOutputs, nInputs, 0 );
      else
        error( 'RtAudio: mxGetAudioApis mex function not found!' );
      end
    end

  end % static methods
  
  methods
  
    %-----------------------------------------------------------
    function obj = RtAudio( api )
      if exist( 'mxGetAudioDeviceCount', 'file' ) ...
          && exist( 'mxGetAudioApis', 'file' ) ...
          && exist( 'mxGetAudioDeviceName', 'file' ) ...
          && exist( 'mxGetAudioDeviceInfo', 'file' ) ...
          && exist( 'mxGetAudioDeviceIds', 'file' ) ...
          && exist( 'mxControlAudioStream', 'file' )
        obj.isValid = true;
      else
        error( 'RtAudio: Some or all mex functions not found!' );
      end
      % Check that we have compiled APIs
      apis = obj.getAudioApis(0, 0);
      if isempty( apis )
        error( 'RtAudio: No compiled audio APIs found!' );
      end
      if nargin == 0
        % If no api specified, use the first one compiled
        obj.api_ = char( apis( 1 ) );
      else
        if ischar( api )
          if any( strcmp( apis, api ))
            obj.api_ = char( api );
          else
            error( 'RtAudio: Api input argument is invalid!' );
          end
        else
          error( 'RtAudio: Api input argument must be a string!' );
        end
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
    function list = listDevices( obj, show, api )
      if nargin < 2, show = false; end
      if nargin < 3, api = obj.api_; end
      inputIds = obj.getAudioDeviceIds( 0, 1, api );
      outputIds = obj.getAudioDeviceIds( 1, 0, api );
      list = '';
      for n = 1:length( outputIds )
        if n == 1
          list = sprintf( 'Output Devices: \n' );
        end
        name = sprintf( '\t%d: %s\n', n, obj.getAudioDeviceName( outputIds( n ) ) );
        list = [list name]; %#ok<*AGROW>
      end
      for n = 1:length( inputIds )
        if n == 1
          list = [list sprintf( 'Input Devices: \n' ) ];
        end
        name = sprintf( '\t%d: %s\n', n, obj.getAudioDeviceName( inputIds( n ) ) );
        list = [list name];
      end
      if show
        fprintf( '%s', list );
      end
    end


    %-----------------------------------------------------------
    function count = getAudioDeviceCount( obj, nOutputs, nInputs, api )
      if nargin < 4, api = obj.api_; end
      if nargin < 3, nInputs = 0; end
      count = 0;
      try
        count = mxGetAudioDeviceCount( nOutputs, nInputs, 0, api );
      catch
        error('RtAudio.m: error calling mxGetAudioDeviceCount.');
      end
    end
    
    
    %-----------------------------------------------------------
    function ids = getAudioDeviceIds( obj, nOutputs, nInputs, api )
      if nargin < 4, api = obj.api_; end
      ids = [];
      try
        ids = mxGetAudioDeviceIds( nOutputs, nInputs, 0, api );
      catch
        error('RtAudio.m: error calling mxGetAudioDeviceIds.');
      end
    end
    

    %-----------------------------------------------------------
    function name = getAudioDeviceName( obj, deviceId, api )
      if nargin < 3, api = obj.api_; end
      name = '';
      try
        name = mxGetAudioDeviceName( deviceId, api );
      catch
        error('RtAudio.m: error calling mxGetAudioDeviceName.');
      end
    end

    
    %-----------------------------------------------------------
    function info = getAudioDeviceInfo( obj, deviceId, api )
      if nargin < 3, api = obj.api_; end
      info = struct( 'name', [], 'outputChannels', [], ...
        'inputChannels', [], 'duplexChannels', [], 'sampleRates', [] );
      try
        info = mxGetAudioDeviceInfo( deviceId, api );
      catch
        error('RtAudio.m: error calling mxGetAudioDeviceInfo.');
      end
    end
    
    
    %-----------------------------------------------------------
    function startAudioStream(obj, oChannels, iChannels, sampleRate, ...
                              iDuration, oSignal, nRepetitions, ...
                              oDevice, iDevice, triggerThreshold, ...
                              triggerChannel, triggerTimeout, triggerBackup, ...
                              oChannelOffset, iChannelOffset, nBufferFrames, api)
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
      if nargin < 17, api = obj.api_; end

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
          oChannelOffset, iChannelOffset, nBufferFrames, api );
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