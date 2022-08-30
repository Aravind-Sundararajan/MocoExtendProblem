function goal_tree = construct_goal_tree()
%% Extract only those that are directories.
    goals = get_goal_names("custom_goals");
    %% construct tree of goals, setter functions, dtypes and args
    goal_tree = struct();
    for goal = goals
        libPath = pwd+"/custom_goals/"+goal+"/"+goal+".h";
        setters = get_setter_functions(libPath);
        goal_tree.(goal) = struct();
        for s = string(fields(setters))'
            goal_tree.(goal).(s) = setters.(s);
        end
    end
end