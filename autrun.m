OPtoolsWorkDir=fileparts(mfilename('fullpath'));

addpath(fullfile(OPtoolsWorkDir,'sim'));
addpath(fullfile(OPtoolsWorkDir,'post'));


warning('off','MATLAB:interpn:NaNstrip');
disp('> MATLAB:interpn:NaNstrip warning disabled..')


% make sure you also add
% https://github.com/pusch-research/PMtools
% https://github.com/pusch-research/matlab-toolbox

clear
