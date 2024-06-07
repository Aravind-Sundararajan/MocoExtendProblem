function out = mex_end()
% mex_end   short script for adding closing paren. for namespace to
% finalize mex interface definition.
%   out = mex_end() 
%
%   
%   See also cpp_start, cpp_end, mex_start, wrap_start, wrap_end

out = "";
%% MEX DEF END
mex_def_end =[...
    "",...
    "} //namespace",...
    "",...
    "MEX_DISPATCH",...
    ];
for line = mex_def_end
    out = out + line + newline;
end
end