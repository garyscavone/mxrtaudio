% This directory contains a variety of files that provide RtAudio support
% from within the Matlab environment.
%
% by Gary P. Scavone, McGill University, 2019-2020.

% Files and directories:
% - RtAudio.m: a Matlab class that simplifies the RtAudio MEX file usage
% - mxcompile.m: script to compile MEX functions on a given OS/API
% - mxclean.m: script to cleanup directory after compiling
% - mxtest.m: script to test RtAudio class functionality
% - scope.m: script to test continuous input and display functionality
% - include: RtAudio.h header file
% - src: RtAudio.cpp, 4 MEX files that provide RtAudio support
% - asio: C++ header and source files needed to compile ASIO support

% The compiled MEX files will reside in the top level directory. One can
% make use of these files, as well as the RtAudio Matlab class, by adding
% this directory to the Matlab path on a given system.

