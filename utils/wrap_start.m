function out = wrap_start()
% wrap_start returns string with start of MATLAB class definition.
%   out = wrap_start() 
%
%
%   See also cpp_start, cpp_end, mex_start, mex_end, wrap_end

out = "";
mat_def_start = ...
    "classdef extend_problem < handle" + newline + ...
    "    " + newline + ...
    "    properties (Access = private)" + newline + ...
    "        id_ % ID of the session" + newline + ...
    "    end" + newline + ...
    "    " + newline + ...
    "    methods" + newline + ...
    "        function this = extend_problem(Cptr)" + newline + ...
    "            %extendProblem Create a new" + newline + ...
    "            disp(Cptr)" + newline + ...
    "            assert(isinteger(Cptr));" + newline + ...
    "            this.id_ = extendProblem('new',Cptr);" + newline + ...
    "        end" + newline + ...
    "        " + newline + ...
    "        function delete(this)" + newline + ...
    "            %delete Destructor" + newline + ...
    "            extendProblem('delete', this.id_);" + newline + ...
    "        end";
out = out + mat_def_start + newline;
end