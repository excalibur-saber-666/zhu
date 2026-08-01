function root = stage1_project_root()
%STAGE1_PROJECT_ROOT Return the flat project directory independent of current folder.

root = fileparts(mfilename('fullpath'));
end
