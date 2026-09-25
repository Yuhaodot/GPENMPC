function result = run_native_lower_loop_readonly_host_tests(outputDir)
% Tests the optional read-only selector extension. NEVER calls live=true.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir),'m600check:ExistingOutput','Use a new output directory.');
here=fileparts(mfilename('fullpath'));addpath(here);mkdir(outputDir);
testPath=fullfile(outputDir,'NEVER_OPENED.json');
result=inspect_native_lower_loop_readonly(testPath,false);
assert(result.passed&&result.COM_open==0&&~isfile(testPath)&&~isfile(testPath+".preflight.json"));
result.files_written_by_tested_entry=0;
fid=fopen(fullfile(outputDir,'HOST_RESULT.json'),'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));disp(jsonencode(result,PrettyPrint=true));
end
