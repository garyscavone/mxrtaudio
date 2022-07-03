% mxcompile.m
%
% Makefile-like script to compile the mxAudio routines on various
% platforms. In general, the MEX compilation process can be frustrating,
% requiring the availability of the same compiler used to compile the
% version of Matlab being used. Try typing 'mex -setup' in the Matlab
% command window to see if a compiler is found.
%
% In Windows, this has been tested with the Mingw64 compiler and Visual
% Studio. If there is a problem compiling in Windows, it is most likely
% related to the path to the ole32, winmm, and dsound libraries.
%
% by Gary P. Scavone, McGill University, 2018-2022.

objfile = 'RtAudio.o';
flags = {'-g', '-Iinclude'};
libs = '';

if ismac
  api = [1 2]; % 1 = CoreAudio, 2 = Jack
  if sum( api == 1 )
    flags = [flags, {'-D__MACOSX_CORE__'}];
    libs = 'LDFLAGS=\$LDFLAGS -framework CoreAudio -framework CoreMidi -framework CoreFoundation';
  end
  if sum( api == 2 )
    flags = [flags, {'-D__UNIX_JACK__', '-I/usr/local/include/', '-ljack', '-lpthread'}];
  end
elseif ispc
  api = [1 2]; % 1 = DS, 2 = ASIO
  objfile = '*.obj';
  flags = [flags, {['-L' fullfile(matlabroot,'sys','lcc64','lcc64','lib64')], '-lole32'}];
  if sum( api == 1 )
    flags = [flags, {'-D__WINDOWS_DS__', '-lwinmm', '-ldsound'}];
  end
  if sum( api == 2 )
    flags = [flags, {'-D__WINDOWS_ASIO__', '-Iasio'}];
    mex( flags{:}, '-c', 'asio/*.cpp' );
  end
elseif isunix
  flags = [flags, {'-D__LINUX_ALSA__'}];
  libs = 'LDFLAGS=\$LDFLAGS -lasound -lpthread';
end

mex( flags{:}, '-c', 'src/RtAudio.cpp' );
mex( flags{:}, 'src/mxGetAudioApis.cpp', objfile, libs );
mex( flags{:}, 'src/mxGetAudioDeviceCount.cpp', objfile, libs );
mex( flags{:}, 'src/mxGetAudioDeviceIds.cpp', objfile, libs );
mex( flags{:}, 'src/mxGetAudioDeviceName.cpp', objfile, libs );
mex( flags{:}, 'src/mxGetAudioDeviceInfo.cpp', objfile, libs );
mex( flags{:}, 'src/mxControlAudioStream.cpp', objfile, libs );
