function root=canonicalAssetRoot()
% Return the canonical asset directory in this HIL distribution.
distribution=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
root=string(fullfile(distribution,'assets','canonical'));
end
