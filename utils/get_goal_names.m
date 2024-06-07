function goalNames = get_goal_names(goaldir)
% get_goal_names returns list of goals in the custom_goals dir based on the folder names.
%   goalNames = get_goal_names() 
%
%
%   See also get_setter_functions

files = dir(goaldir);
dirFlags = [[files.isdir] & ~ismember({files.name},{'.','..'})];
files = files(dirFlags);
goalNames = {files.name};
goalNames = string(goalNames);
end