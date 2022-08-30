function build_extend_class(cppName,wrapName)
%% SETUP
goal_tree = construct_goal_tree();
[c,m,w] = generate_content(goal_tree);
%% CPP class construction
% construct extendProblem.cpp
s = "";
s = cpp_start(string(fields(goal_tree))');
s = s + c + newline;
s = s + cpp_end() + newline;

% construct mex_dispatch info
s = s + mex_start() + newline;
s = s + m + newline;
s = s + mex_end() + newline;

% write to file
fid= fopen(cppName,'w');
fprintf(fid,'%s\n',s);
fclose(fid);
%% MATLAB class construction
s = "";
s = wrap_start();
s = s + w + newline;
s = s + wrap_end();
fid= fopen(wrapName,'w');
fprintf(fid,'%s\n',s);
fclose(fid);
end