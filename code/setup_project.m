function root = setup_project()
%SETUP_PROJECT Add only the active code directory to the MATLAB path.
%   ROOT = SETUP_PROJECT resolves paths from this file, so calls do not
%   depend on MATLAB's current working directory.

code_directory = fileparts(mfilename('fullpath'));
root = fileparts(code_directory);
addpath(code_directory);
rehash;
end
