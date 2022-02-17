function mxclean( ~ )
  delete *.o*
  if nargin > 0
    delete *.mex*
  end
end