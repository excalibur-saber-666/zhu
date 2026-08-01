function root = setup_project()
%SETUP_PROJECT Add the flat project directory to the MATLAB path.
%   ROOT = SETUP_PROJECT resolves paths from this file, so calls do not
%   depend on MATLAB's current working directory.

root = fileparts(mfilename('fullpath'));
addpath(root);
rehash;
end
