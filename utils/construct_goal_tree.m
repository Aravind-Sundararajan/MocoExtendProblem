function goal_tree = construct_goal_tree(opensimInstallDir)
% construct_goal_tree  makes a struct of member functions found parsing
% custom goal folders depending on the opensim version.
%   goal_tree = construct_goal_tree(opensimInstallDir) 
%
%
%   See also build_extend_class

%% Extract only those that are directories.
goalsdir = "custom_goals_compat";
if contains(opensimInstallDir, "4.5")
    goalsdir = "custom_goals";
end
goals = get_goal_names(goalsdir);

%% construct tree of goals, setter functions, dtypes and args
goal_tree = struct();
for goal = goals
    libPath = pwd+"/" +goalsdir + "/"+goal+"/"+goal+".h";
    setters = get_setter_functions(libPath);
    goal_tree.(goal) = struct();
    for s = string(fields(setters))'
        goal_tree.(goal).(s) = setters.(s);
    end
end
end