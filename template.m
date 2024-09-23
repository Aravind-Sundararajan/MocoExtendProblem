function template(goal_name)
    goaldir = "Moco" + goal_name + "Goal";
    goal_cpp = goaldir + ".cpp";
    goal_h = goaldir + ".h";
    registertype_cpp = "RegisterTypes_osim" + goaldir + ".cpp";
    registertype_h = "RegisterTypes_osim" + goaldir + ".h"
    dll_h = "osim" + goaldir + "DLL.h"

    if ~exist(fullfile("custom_goals",goaldir))
        mkdir(fullfile("custom_goals",goaldir));
    
        edit(fullfile("custom_goals",goaldir,goal_cpp));
        edit(fullfile("custom_goals",goaldir,goal_h));
        edit(fullfile("custom_goals",goaldir,registertype_cpp));
        edit(fullfile("custom_goals",goaldir,registertype_h));
        edit(fullfile("custom_goals",goaldir,dll_h));
    end

end