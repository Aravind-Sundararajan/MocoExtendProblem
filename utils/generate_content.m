function [cpp_out,mex_out,wrap_out] = generate_content(goal_tree) 
%% create mex functions
mex_out = "";
cpp_out = "";
wrap_out = "";
    for goal = string(fields(goal_tree))'
        libPath = pwd+"/custom_goals/"+goal+"/"+goal+".h";
        setters = get_setter_functions(libPath);
        %each goal has a name and weight, though things like path constraints don't have a weight.
        goalfun = "    void add"+ goal +"(const string& goalName, double weight";
        mex_add_def = ...
            "    MEX_DEFINE(add"+goal+") (int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])"+newline+...
            "    {"+newline+...
            "        InputArguments input(nrhs, prhs, "+num2str(length(fields(setters))+3)+");"+newline+...
            "        OutputArguments output(nlhs, plhs, 0);"+newline+...
            "        extendProblem* engine = Session<extendProblem>::get(input.get(0));"+newline+...
            "        engine->add"+goal+"(input.get<string>(1),input.get<double>(2)";
        f_wrap = "        function add"+goal+"(this,goalname,weight";
        wrapfun = "            extendProblem('add"+goal+"', this.id_,goalname,weight";
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
        f_wrap = f_wrap + ")";
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
        
        for s = string(fields(set_calls))'
            cpp_out = cpp_out +set_calls.(s) + newline;
        end
        
        cpp_out = cpp_out +"    }" + newline;
    end
    
end