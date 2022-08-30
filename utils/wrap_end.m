function out = wrap_end()
out = "";
%% MAT END
mat_def_end = ...
    "    end" + newline + ...
    "end";
out = out + mat_def_end + newline;