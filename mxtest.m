r% Test script using the RtAudio matlab class.
%
% by Gary Scavone, McGill University, 2018-2022.

clear all;
apis = RtAudio.getAudioApis(0, 0);
nApi = 1;
if length(apis) < nApi
  disp('!!API specification problem!!');
  return;
end
rth = RtAudio( char(apis(nApi)) );
rth.listDevices( true );

% Basic stream settings
sampleRate = 48000;
oChannels = 0;
iChannels = 2;
oDuration = 2.0;  % seconds (may change depending on other parameters)
nRepetitions = 2; % repetitions of the output signal (after first time)
iDuration = 4.0;
iDevice = 1; % first device = 1 (valid devices meeting requirements)
oDevice = 2; % first device = 1 (valid devices meeting requirements)

% Check that device values are valid
ids = rth.getAudioDeviceIds( 0, iChannels );
if length(ids) < iDevice
  disp('!!Input device specification problem!!');
  return;
end
iDeviceID = ids( iDevice );
ids = rth.getAudioDeviceIds( oChannels, 0 );
if length(ids) < oDevice
  disp('!!Output device specification problem!!');
  return;
end
oDeviceID = ids( oDevice );

% Trigger settings (when iChannels > 0)
triggerThreshold = 0.0; % if this > 0, plotting will be blocked until trigger
triggerChannel = 0;
triggerBackup = 55;
triggerTimeout = 2;  % seconds until stream automatically stopped without trigger received

% Output signal settings (when oChannels > 0)
source = [];
if oChannels
  % Create output signal ... for sines, make integer number of periods to
  % avoid clicks.
  sourceFrames = ceil(sampleRate * oDuration);
  zeroPadFrames = 0; % extra frames added to end of output signal
  doSine = 1; % 1 = sine wave, otherwise use noise
  if doSine
    oFrequency = 220;
    P = round( sampleRate / oFrequency ); % integer period in samples
    sourceFrames = P * round( sourceFrames / P );
    source = zeros( oChannels, sourceFrames+zeroPadFrames );
    for n = 1:oChannels
      source(n, :) = [0.8*sin(2*pi*(0:sourceFrames-1)/P), zeros(1, zeroPadFrames)];
      oFrequency = 2*oFrequency;
      P = round( sampleRate / oFrequency ); % period in samples
    end
  else
    source = [0.8*2*rand(oChannels, sourceFrames)-1, zeros(oChannels, zeroPadFrames)];
  end
  sourceFrames = sourceFrames + zeroPadFrames;
end

% Setup for input signal display
if iChannels
  tmp = 0;
  if oChannels
    tmp = (nRepetitions+1) * sourceFrames;
  end
  if iDuration > 0
    responseFrames = ceil( iDuration * sampleRate );
  else
    responseFrames = ceil( 2.0 * sampleRate ); % arbitrary value
  end
  if tmp > responseFrames
    responseFrames = tmp;
  end
  
  nGetFrames = 4096; % # of frames to retrieve per call
  nTotalFrames = responseFrames;
  x = (0:nTotalFrames-1)/sampleRate;
  y = zeros( iChannels, nGetFrames );
  yt = zeros( iChannels, nTotalFrames );
  h = plot( x, yt.' );
  ylim([-0.2 0.2])
  xlim([0 max(x)]);
  xlabel('Time (seconds)');
end

% Open and start the stream
rth.startAudioStream( oChannels, iChannels, sampleRate, iDuration, ...
  source, nRepetitions, oDeviceID, iDeviceID, triggerThreshold, ...
  triggerChannel, triggerTimeout, triggerBackup );
disp('stream started');

if iChannels
  nPlots = floor(responseFrames / nGetFrames);
  for n = 1:nPlots
    retval = rth.getAudioData( y );
    if ( retval > 0 )
      disp('error getting data ... trigger timeout?');
      break;
    end
    yStart = (n-1)*nGetFrames + 1;
    yt(:,yStart:yStart+nGetFrames-1) = y;
    set( h, {'YData'}, num2cell(yt, 2) );
    drawnow;
  end
end

rth.closeAudioStream();
disp('stream closed');

clear rth;
disp('RtAudio instance cleared');
