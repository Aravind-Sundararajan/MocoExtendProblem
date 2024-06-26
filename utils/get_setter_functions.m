function f = get_setter_functions(hppPath)
% get_setter_functions   uses regex to find the setter methods based on the
% hppPath
%   
%
%   
%   See also cpp_start, cpp_end, mex_start, wrap_start, wrap_end


f = struct();
libPath_data = readlines(hppPath);
ids = intersect(...
    find(contains(libPath_data, 'void')),...
    find(contains(libPath_data, 'set'))...
    );
for i = 1:length(ids)
    [match, nomatch] = regexp(libPath_data{ids(i)},'\(([^)]+)\)','match','split');
    fun = strrep(strrep(nomatch{1},'void',''),' ','');
    arg = match{1};
    arg = split(strrep(strrep(strrep(arg,')',''),'(',''),'const',''),' ');
    arg = arg(~cellfun(@isempty, arg));
    f.(fun) = arg;
end
end