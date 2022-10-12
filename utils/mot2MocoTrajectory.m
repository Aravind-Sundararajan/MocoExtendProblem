function mot2MocoTrajectory(fileName)
data = ReadOpenSimData(fileName);
%% 
%use this _rel.mot file to pull headers. 
%    B = importdata('./s01/S01_Rel_T09_rel.mot');
    %get column headers
    %labels = B.textdata(7:7:end);
labels =data.labels;
%Print other headers:
fid = fopen(fileName,'w');
fprintf(fid,'%s\n','GRF SIM MOCO');
fprintf(fid,'%s\n','version=1');
fprintf(fid,'%s\n',['nRows=',num2str(size(data.data,1))]);
fprintf(fid,'%s\n','nColumns=19');
fprintf(fid,'%s\n','num_controls=0');
fprintf(fid,'%s\n','num_derivatives=0');
fprintf(fid,'%s\n','num_iterations=0');
fprintf(fid,'%s\n','num_multipliers=0');
fprintf(fid,'%s\n','num_parameters=0');
fprintf(fid,'%s\n','num_slacks=0');
fprintf(fid,'%s\n','num_states=18');
fprintf(fid,'%s\n','inDegrees=yes');
fprintf(fid,'%s\n','endheader');

%Print the markers labels:
for i = 1:length(labels)
    txt = labels(i);
    fprintf(fid, '%s\t',char(txt) );
end
fprintf(fid,'\n'); %print an additional empty line
fclose(fid); 

dlmwrite(fileName, data.data,'-append','delimiter','\t','newline','pc')

end