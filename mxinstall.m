function mxinstall( path )
  if isempty( path ), return; end
  copyfile( '*.mex*', path );
end