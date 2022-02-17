% NOTES ON USE OF RTAUDIO IN MEX FUNCTIONS AND EXTERNAL LIBRARIES
% 
% - The biggest limitation in embedding RtAudio in a Mex function is that
%   the audio callback function cannot make use of any Mex functions. Thus,
%   we cannot attempt to pass data directly from the callback back to the
%   Matlab client. The result is that we need to create an internal buffer
%   that we write input data to.
%
%   The approach used here is to support either fixed-duration calls, such
%   that internal buffers are created to hold the full input or output
%   signals, or continuous input streaming (for which a circular internal
%   input buffer of about 2.0 seconds is used). This approach allows data
%   to be returned to the Matlab interface nearly in "real time," as well
%   as providing a blocking mechanism in the Matlab script when retrieving
%   data. The stream can be stopped at any time before the maximum
%   duration, so it behaves nearly identically to the daq and simple
%   built-in audio interfaces.
%
% - COMPILING: In general, compiling either mex or external libraries for
%   use in Matlab can be problematic, as a compliant compiler for the
%   current version of Matlab must be available.

% We have 3 possible ways to use RtAudio inside Matlab:
%
% 1. We can compile MEX functions with RtAudio support. These
%    calls appear as normal Matlab functions, so this is the easiest
%    approach in terms of incorporating IO support directly in the
%    code. The main problem is related to finding a working compiler
%    for older versions of Matlab.
%
% 2. We can compile an external library that provides a series of
%    functions similar to the above. This approach makes it possible to
%    pass data directly from Matlab variables to the library functions and
%    also allows periodic returning of audio input data for display during
%    recording. This approach was used for 'zaudio' but is no longer
%    maintained.
%
% 3. We can compile separate command-line cpp programs that can be
%    called from within Matlab using the system() function. This is
%    perhaps the most basic approach but it is the most likely
%    approach to compile without much trouble. For audio IO, files
%    have to be read/written and loaded back into Matlab after
%    execution is complete. This approach was used for 'zaudio' but
%    is no longer maintained.
