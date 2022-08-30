function goalNames = get_goal_names(goaldir)
files = dir(goaldir);
dirFlags = [[files.isdir] & ~ismember({files.name},{'.','..'})];
files = files(dirFlags);
goalNames = {files.name};
goalNames = string(goalNames);
end