function out = mex_end()
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