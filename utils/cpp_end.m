function out = cpp_end()
% cpp_end returns final lines for cpp class definition.
%   out = cpp_end() 
%
%
%   See also cpp_start, mex_start, mex_end, wrap_start, wrap_end


out = "";
%% CLASS DEF END
class_def_end = [...
    "private:",...
    "    OpenSim::MocoProblem *m_p = NULL;",...
    "};",...
    ];
for line = class_def_end
    out = out + line + newline;
end
end