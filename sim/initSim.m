% run this init script in your main file

% INPUT
% the following inputs are needed (all strings, may be defined for different machines/users/etc. - see getPath.m)
% paths=struct();
% paths.specs
% paths.CpCqCt
% paths.FSTtmplInputFile
% paths.FSTnewInputFileDir
% paths.FSTbin
% paths.resultsTmpDir
% paths.parforProgressFile (not needed here but later)
% paths.results (not needed here but later)

% OUTPUT
% simOptTmpl (struct with default options)


%% add OpenFAST

% remove paths of all OpenFAST binaries
searchPath_arr=strsplit(path,';');
rmPath_iArr=find(stroverlap(searchPath_arr,fullfile('Model','bin_OpenFAST'))); % assume OpenFAST binaries are stored in these kind of folders
for ii=rmPath_iArr
    rmpath(searchPath_arr{ii})
end
clear searchPath_arr rmPath_iArr

% add OpenFAST binaries
addpath(getPath(paths.FSTbin)) % add OpenFAST binaries


%% init

% check if tmp directories exist and create if not
tmpModelDir=getPath(paths.FSTnewInputFileDir);
if ~exist(tmpModelDir,"dir")
    mkdir(tmpModelDir);
    disp(['> ' tmpModelDir ' folder created..'])
end
tmpResultDir=getPath(paths.resultsTmpDir);
if ~exist(tmpResultDir,"dir")
    mkdir(tmpResultDir);
    disp(['> ' tmpResultDir ' folder created..'])
end

% copy ALL files from templates folder to new model folder
tmplDir=fileparts(getPath(paths.FSTtmplInputFile));

fileName_arr=dir(tmplDir);
for ii=3:length(fileName_arr)
    if fileName_arr(ii).isdir
        [status,msg]=copyfile(fullfile(tmplDir,fileName_arr(ii).name),...
                              fullfile(tmpModelDir,fileName_arr(ii).name));
    else
        [status,msg]=copyfile(fullfile(tmplDir,fileName_arr(ii).name),tmpModelDir);
    end
    if status==0
        error(msg);
    end
end

% load specs
[specsFilePath,specsFileName,specsFileExt]=fileparts(getPath(paths.specs));
if strcmp(specsFileExt,'.m')
     % executeable matlab function
     addpath(specsFilePath);
     specs=feval(specsFileName,getPath(paths.FSTtmplInputFile),getPath(paths.CpCqCt));
elseif strcmp(specsFileExt,'.mat')
    % pre-saved mat file
    specs=load(getPath(paths.specs)); 
else
    error('not implemented.')
end

% options for simulation
simOptTmpl=struct(); 
simOptTmpl.disp=2; % {0: no disp, 1: only important disp, 2: informative disp, 3: disp all}
% OpenFast (FST)
simOptTmpl.FSTdataTmpl=struct(); % FST parameters which should be overwritten (to be defined study specific)
simOptTmpl.FSTdataTmpl.TMax=9999; % not sure, but this may be the max. allowed sim time from openfast
simOptTmpl.FSTnewInputFile=[]; % will be set in the loop
simOptTmpl.FSTtmplInputFile=getPath(paths.FSTtmplInputFile);
simOptTmpl.FSTnewInputFileDir=getPath(paths.FSTnewInputFileDir);
simOptTmpl.FSTnewModelSuffix=''; % to be overwritten during optimization
FSTinitFileName=generateFASTinFiles(simOptTmpl.FSTtmplInputFile,...
                                    simOptTmpl.FSTnewInputFileDir,...
                                    simOptTmpl.FSTnewModelSuffix);
[simOptTmpl.FSToutNameArr,simOptTmpl.SLsmplTime]=getFASTparamFromSL(FSTinitFileName);

for i_out=1:numel(simOptTmpl.FSToutNameArr)
    simOptTmpl.idx.y.(strclean(simOptTmpl.FSToutNameArr{i_out}))=i_out;  % get indices for outputs
end
% Simulink (SL)
simOptTmpl.SLmodelName='model_constInput';
simOptTmpl.SLmaxTime=simOptTmpl.FSTdataTmpl.TMax; 
% settling detector options
simOptTmpl.settlingSmplTime=simOptTmpl.SLsmplTime;
simOptTmpl.settlingMovAvgWindowLength=20/simOptTmpl.SLsmplTime;
simOptTmpl.settlingAbsThresh=0.01; % to be overwritten depending on nominal rotor speed
simOptTmpl.settlingOutName={'RotSpeed'}; % name(s) of output(s) which should settle
simOptTmpl.settlingMinTime=80; % 10 for testing!
simOptTmpl.settlingMaxTime=500; % timeout to wait for a signal to settle
% output limitation check (to automatically stop simulation)
simOptTmpl.limCheckOutName={'RotSpeed'};
simOptTmpl.limCheckLowerBound=[0];
simOptTmpl.limCheckUpperBound=[specs.schedule.wr_rated_radDs*radDs2rpm*2]; % simOptTmpl.limCheckUpperBound=[specs.wr_rated_rpm*2];
% pre-compute indices
simOptTmpl.n_out=length(simOptTmpl.FSToutNameArr);
simOptTmpl.idx.y.settling=findselection(simOptTmpl.settlingOutName,simOptTmpl.FSToutNameArr);
simOptTmpl.idx.y.limCheck=findselection(simOptTmpl.limCheckOutName,simOptTmpl.FSToutNameArr);
% define names
simOptTmpl.name.y=[simOptTmpl.FSToutNameArr]; % all output names (FST and SL)
simOptTmpl.name.u={'BldPitchC' 'GenTq'}; % control inputs {'BldPitchC', 'GenTq','BldPitchD','BldPitchQ',...}; to be overwritten depending on test case
simOptTmpl.name.p={};% design parameters;  to be overwritten depending on study


warning('off','MATLAB:mir_warning_maybe_uninitialized_temporary')
clear tmplDir tmpModelDir fileName_arr msg status specsFileName specsFileExt specsFilePath ii i_out FSTinitFileName tmpResultDir
