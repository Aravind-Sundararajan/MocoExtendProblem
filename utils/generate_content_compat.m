function [cpp_out,mex_out,wrap_out] = generate_content_compat(goal_tree)
% generate_content_compat  Procedurally constructs the matlab wrapper, the
% mex interface and the cpp class with special consideration for OpenSim pre-4.5.
%   [cpp_out,mex_out,wrap_out] = generate_content_compat(A)
%
%   See also construct_goal_tree, generate_content

%% create mex functions
disp("detected using opensim 4.4 or lower.");
mex_out = "";
cpp_out = "";
wrap_out = "";
for goal = string(fields(goal_tree))'
    libPath = pwd+"/custom_goals_compat/"+goal+"/"+goal+".h";
    setters = get_setter_functions(libPath);
    %each goal has a name and weight, though things like path constraints don't have a weight.
    goalfun = "    void add"+ goal +"(const std::string& goalName, double weight";
    mex_add_def = ...
        "    MEX_DEFINE(add"+goal+") (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])"+newline+...
        "    {"+newline+...
        "        InputArguments input(nrhs, prhs, "+num2str(length(fields(setters))+3)+");"+newline+...
        "        OutputArguments output(nlhs, plhs, 0);"+newline+...
        "        extendProblem* engine = Session<extendProblem>::get(input.get(0));"+newline+...
        "        engine->add"+goal+"(input.get<std::string>(1),input.get<double>(2)";
    f_wrap = "        function add"+goal+"(this,goalName,weight";
    wrapfun = "            extendProblem('add"+goal+"', this.id_,goalName,weight";
    set_calls = struct();
    i = 1;
    for s = string(fields(setters))'
        mex_add_def = mex_add_def + ",input.get<"+setters.(s){1}+">("+num2str(i+2)+")";
        i = i + 1;
        goalfun = goalfun + ", " +setters.(s){1} + " "+ setters.(s){2};
        set_calls.(s) = "        goal->"+s+"("+setters.(s){2}+");";
        f_wrap =f_wrap + ", "+setters.(s){2};
        wrapfun = wrapfun + ", "+setters.(s){2};
    end

    %MATLAB class def
    f_wrap = f_wrap + ")" + newline + "           assert(isscalar(weight))";
    wrapfun = wrapfun + ");";
    
    % Generate function documentation
    doc_string = newline + sprintf("        %% add%s Add a %s goal to the problem", goal, goal) + newline;
    doc_string = doc_string + sprintf("        %%   Adds a %s goal with specified parameters", goal) + newline;
    doc_string = doc_string + "        %" + newline;
    doc_string = doc_string + "        %   Parameters:" + newline;
    doc_string = doc_string + "        %      goalName  - Name of the goal (string)" + newline;
    doc_string = doc_string + "        %      weight    - Weight of the goal (scalar)" + newline;
    
    % Add documentation for setter parameters
    for s = string(fields(setters))'
        doc_string = doc_string + sprintf("        %      %s - %s parameter", ...
            setters.(s){2}, s) + newline;
    end
    
    wrap_out = wrap_out + f_wrap;
    wrap_out = wrap_out + doc_string;
    wrap_out = wrap_out + newline + "           assert(isscalar(weight))" + newline;
    wrap_out = wrap_out + wrapfun + newline;
    wrap_out = wrap_out + "        end" + newline;

    %mex
    goalfun = goalfun + "){";
    mex_add_def = mex_add_def + ");"+newline+"    }";
    mex_out = mex_out + mex_add_def + newline;

    %cpp class def
    cpp_out = cpp_out +goalfun + newline;
    cpp_out = cpp_out +"        mexPrintf(""Adding "+goal+" goal\n"");" + newline;
    cpp_out = cpp_out +"        auto* goal = m_p->addGoal<"+goal+">(goalName, weight);" + newline;
    cpp_out = cpp_out +"        goal->setName(goalName);" + newline;
    for s = string(fields(set_calls))'
        cpp_out = cpp_out +set_calls.(s) + newline;
    end

    cpp_out = cpp_out +"    }" + newline;
end

end