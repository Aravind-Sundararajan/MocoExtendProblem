function goal_tree = construct_goal_tree(opensimInstallDir)
%% Extract only those that are directories.
    goalsdir = "custom_goals";
    if contains(opensimInstallDir, "4.5")
        goalsdir = "custom_goals45";
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