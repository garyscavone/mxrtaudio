% This script implements a simple display of continuous realtime audio
% input using the RtAudio matlab class.
%
% by Gary Scavone, McGill University, June 2020-2022.

clear all;
apis = RtAudio.getAudioApis(0, 0);
nApi = 1;
if length(apis) < nApi
  disp('!!API specification problem!!');
  return;
end
rth = RtAudio( char(apis(nApi)) );
rth.listDevices( true );

iDuration = -1.0; % negative value for continuous input
sampleRate = 48000;
oChannels = 0;
iChannels = 2;
iDevice = 1; % first device = 1 (first valid device meeting requirements)
oDevice = [];

% Check that device values are valid
ids = rth.getAudioDeviceIds( 0, iChannels );
if length(ids) < iDevice
  disp('!!Input device specification problem!!');
  return;
end
iDeviceID = ids( iDevice );

nGetFrames = 2048; % must be < 0.5 seconds at this sample rate
nBlocks = 80;      % arbitrary value
nTotalFrames = nGetFrames * nBlocks;
x = 0:nTotalFrames-1;
y = zeros( iChannels, nGetFrames );
yt = zeros( iChannels, nTotalFrames );
h = plot( x, yt.' );
xlabel('Time (samples)')
xlim([0 max(x)]);
ylim([-0.2 0.2]);

% Use a button to kill the process
dialogBox = uicontrol('Style', 'PushButton', 'String', 'Break', ...
  'Callback', 'delete(gcbf)');

rth.startAudioStream( oChannels, iChannels, sampleRate, iDuration, ...
  [], 0, 0, iDeviceID );
disp('stream started');

fCounter = 1;
% for n = 1:100
%   rth.getAudioData( y );
%   yStart = (fCounter-1)*nGetFrames + 1;
%   yt(:,yStart:yStart+nGetFrames-1) = y;
%   set( h, {'YData'}, num2cell(yt, 2) );
%   drawnow;
%   fCounter = fCounter + 1;
%   if fCounter > nBlocks
%     fCounter = 1;
%     yt(:) = 0;
%   end
% end
% 
% rth.stopAudioStream();
% disp('stream stopped');
% 
% rth.restartAudioStream();
% disp('stream restarted ... use Break button to stop.');

while ishandle(dialogBox)
  rth.getAudioData( y );
  yStart = (fCounter-1)*nGetFrames + 1;
  yt(:,yStart:yStart+nGetFrames-1) = y;
  set( h, {'YData'}, num2cell(yt, 2) );
  drawnow;
  fCounter = fCounter + 1;
  if fCounter > nBlocks
    fCounter = 1;
    yt(:) = 0;
  end
end

rth.stopAudioStream();
disp('stream stopped');

rth.closeAudioStream();
disp('stream closed');

clear rth;
disp('RtAudio instance cleared');
