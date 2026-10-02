function root = stage1_project_root()
%STAGE1_PROJECT_ROOT Return the workspace root independent of the current folder.

root = fileparts(fileparts(mfilename('fullpath')));
end
