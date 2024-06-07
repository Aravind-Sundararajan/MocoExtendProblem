function [cpp_out,mex_out,wrap_out] = generate_content(goal_tree)
% generate_content_compat  Procedurally constructs the matlab wrapper, the
% mex interface and the cpp class.
%   [cpp_out,mex_out,wrap_out] = generate_content_compat(goal_tree)
%
%   See also construct_goal_tree, generate_content_compat
%% create mex functions
disp("detected using opensim 4.5.");
mex_out = "";
cpp_out = "";
wrap_out = "";
for goal = string(fields(goal_tree))'
    libPath = pwd+"/custom_goals/"+goal+"/"+goal+".h";
    setters = get_setter_functions(libPath);
    %each goal has a name and weight, though things like path constraints don't have a weight.
    goalfun = "    void add"+ goal +"(const std::string& goalName, double weight,"+...
        "bool div_disp, bool div_dur, bool div_mass";
    mex_add_def = ...
        "    MEX_DEFINE(add"+goal+") (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])"+newline+...
        "    {"+newline+...
        "        InputArguments input(nrhs, prhs, "+num2str(length(fields(setters))+ 3 + 3)+");"+newline+...
        "        OutputArguments output(nlhs, plhs, 0);"+newline+...
        "        extendProblem* engine = Session<extendProblem>::get(input.get(0));"+newline+...
        "        engine->add"+goal+"(input.get<std::string>(1),input.get<double>(2),"+...
        "input.get<bool>(3),input.get<bool>(4),input.get<bool>(5)";
    % Adding 3 parameters for problem_id, function_name, goal_weight
    % Adding 3 more paramaters for div_disp, div_dur, div_mass
    f_wrap = "        function add"+goal+"(this,goalName,weight,"+...
        "div_disp, div_dur,div_mass";
    wrapfun = "            extendProblem('add" + goal + "', this.id_,goalName,weight" + ...
        ", div_disp, div_dur, div_mass";
    set_calls = struct();
    i = 1;
    for s = string(fields(setters))'
        mex_add_def = mex_add_def + ",input.get<"+setters.(s){1}+">("+num2str(i+5)+")";
        i = i + 1;
        goalfun = goalfun + ", " +setters.(s){1} + " "+ setters.(s){2};
        set_calls.(s) = "        goal->"+s+"("+setters.(s){2}+");";
        f_wrap =f_wrap + ", "+setters.(s){2};
        wrapfun = wrapfun + ", "+setters.(s){2};
    end

    %MATLAB class def
    f_wrap = f_wrap + ")" + newline + "           assert(isscalar(weight))";
    wrapfun = wrapfun + ");";
    wrap_out = wrap_out + f_wrap + newline;
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

    % Set all the divide_by properties
    cpp_out = cpp_out +"        goal->setDivideByDisplacement(div_disp);" + newline;
    cpp_out = cpp_out +"        goal->setDivideByDuration(div_dur);" + newline;
    cpp_out = cpp_out +"        goal->setDivideByMass(div_mass);" + newline;

    % Set additional properties for the goal
    for s = string(fields(set_calls))'
        cpp_out = cpp_out +set_calls.(s) + newline;
    end

    cpp_out = cpp_out +"    }" + newline;
end

end