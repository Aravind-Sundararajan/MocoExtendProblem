function out = wrap_end()
% wrap_end returns string with final lines of MATLAB class definition.
%   out = wrap_end() 
%
%
%   See also cpp_start, cpp_end, mex_start, mex_end, wrap_start


out = "";
%% MAT END
mat_def_end = ...
    "    end" + newline + ...
    "end";
out = out + mat_def_end + newline;