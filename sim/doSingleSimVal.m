
% ccc
% load('Results\SOAR\22-04-10_1154_v2f_01.mat')
% load('Results\SOAR\22-04-13_2137_v2f_thirdVary.mat')


%% re-run single simulation for checking


windSpeed_mDs_act=10;
PreCone_deg_act=3.6;
ShftTilt_deg_act=8.4;

% SLin
SLin=struct();
SLin.GenTq=4.6e7;
SLin.BldPitchC=deg2rad(0); % 0 -2
SLin.IPCgain=1.17; % 1.17 1.18
SLin.n_step=1; % number of parameter combinations; assume that x_scaled is a row vector (or array of row vectors)
SLin.extra=[]; % extra inputs (e.g. cable tensions, flaps, etc.)

% simOpt
simOpt=simOptTmpl; % re-initialize (required for parfor)
FSTdata=simOpt.FSTdataTmpl;
FSTdata.InflowFile.HWindSpeed=windSpeed_mDs_act;
FSTdata.EDFile.RotSpeed=interp1(specs.schedule.v_mDs_arr,specs.schedule.wr_radDs_arr,windSpeed_mDs_act)*radDs2rpm;;  % overwrite initial value in ElastoDyn
FSTdata.EDFile.BlPitch_1_=rad2deg(SLin.BldPitchC(1)); % overwrite initial value in ElastoDyn
FSTdata.EDFile.BlPitch_2_=rad2deg(SLin.BldPitchC(1)); % overwrite initial value in ElastoDyn
FSTdata.EDFile.BlPitch_3_=rad2deg(SLin.BldPitchC(1)); % overwrite initial value in ElastoDyn
FSTdata.EDFile.PreCone_1_=PreCone_deg_act;
FSTdata.EDFile.PreCone_2_=PreCone_deg_act;
FSTdata.EDFile.PreCone_3_=PreCone_deg_act;
FSTdata.EDFile.ShftTilt=ShftTilt_deg_act;
FSTdata.TMax=simOpt.SLmaxTime;
simOpt.FSTnewModelSuffix='_tmp';
[simOpt.FSTnewInputFile,tmpFileName_arr]=generateFASTinFiles(...
    simOpt.FSTtmplInputFile,...
    simOpt.FSTnewInputFileDir,...
    simOpt.FSTnewModelSuffix,FSTdata);  
simOpt.settlingAbsThresh=FSTdata.EDFile.RotSpeed^2.5*0.001;

% re-initialize in case OutList/dt changed (from init.m)
[simOpt.FSToutNameArr,simOpt.SLsmplTime]=getFASTparamFromSL(simOpt.FSTnewInputFile); 
for i_out=1:numel(simOpt.FSToutNameArr)
    simOpt.idx.y.(strclean(simOpt.FSToutNameArr{i_out}))=i_out;  % get indices for outputs
end
simOpt.n_out=length(simOpt.FSToutNameArr);
simOpt.idx.y.settling=findselection(simOpt.settlingOutName,simOpt.FSToutNameArr);
simOpt.idx.y.limCheck=findselection(simOpt.limCheckOutName,simOpt.FSToutNameArr);

% open simulink model to view scopes
open(simOpt.SLmodelName);

% run simulation
simout=doSim(simOpt,SLin);