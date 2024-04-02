
% ccc
% load('Results\SOAR\22-04-10_1154_v2f_01.mat')
% load('Results\SOAR\22-04-13_2137_v2f_thirdVary.mat')

%% re-run single simulation for checking

i_wind=1;
i_wind=findselection(10,[res_arr.windSpeed]);
i_GenTq=4;
i_BldPitch=6;



[n_windSpeed,~]=size(res_arr);
[n_GenTqSmpl,n_BldPitchCSmpl]=size(res_arr(1).BldPitchC);
i_outer=i_wind;
res_act=res_arr(i_wind);

% SLin
SLin=struct();
SLin.GenTq=res_act.GenTq(i_BldPitch,i_GenTq);
SLin.BldPitchC=res_act.BldPitchC(i_BldPitch,i_GenTq);
SLin.n_step=1; % number of parameter combinations; assume that x_scaled is a row vector (or array of row vectors)
SLin.extra=[]; % extra inputs (e.g. cable tensions, flaps, etc.)

% simOpt
simOpt=simOptTmpl; % re-initialize (required for parfor)
FSTpar=struct();
FSTpar.InflowFile.HWindSpeed=res_act.windSpeed;
FSTpar.EDFile.RotSpeed=res_act.outAvg((i_GenTq-1)*n_BldPitchCSmpl+i_BldPitch,...
    findselection('RotSpeed',simOpt.FSToutNameArr));  % overwrite initial value in ElastoDyn
FSTpar.TMax=simOpt.SLmaxTime;
simOpt.FSTnewModelSuffix=num2str(['_' num2str(res_act.windSpeed)]);
[simOpt.FSTnewInputFile,tmpFileName_arr]=generateFASTinFiles(...
    simOpt.FSTtmplInputFile,...
    simOpt.FSTnewInputFileDir,...
    simOpt.FSTnewModelSuffix,FSTpar);  
simOpt.settlingAbsThresh=FSTpar.EDFile.RotSpeed^2.5*0.001;
simOpt.settlingMinTime=150;

% re-initialize in case OutList/dt changed (from init.m)
[simOpt.FSToutNameArr,simOpt.SLsmplTime]=getFASTparamFromSL(simOpt.FSTnewInputFile); 
for i_out=1:numel(simOpt.FSToutNameArr)
    simOpt.idx.y.(strclean(simOpt.FSToutNameArr{i_out}))=i_out;  % get indices for outputs
end
simOpt.n_out=length(simOpt.FSToutNameArr);
simOpt.idx.y.settling=findselection(simOpt.settlingOutName,simOpt.FSToutNameArr);
simOpt.idx.y.limCheck=findselection(simOpt.limCheckOutName,simOpt.FSToutNameArr);


% open simulink model to view scopes
open model_constInput

% run simulation
simout=doSim(simOpt,SLin);