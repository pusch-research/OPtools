function simRes=doSim(simOpt,SLin)
        

try

    % load model
    clear mex %#ok<CLMEX>  to avoid total crash which happens sometimes
    isSysLoaded=bdIsLoaded(simOpt.SLmodelName);
    if ~isSysLoaded
        load_system(simOpt.SLmodelName)
    end
    SLsimset=simset('SrcWorkspace','current','DstWorkspace','current');

    % restart simulation until all input steps have been simulated
    simRes=struct('outAvg',[],'outMax',[],'isValid',[],'settlingTime',[],'out',[]);
    while SLin.n_step>0

        % run simulation
        if simOpt.disp>1
            if SLin.n_step==1
                disp(['> start SL sim with BldPitchC=' num2str(rad2deg(SLin.BldPitchC)) ...
                      'deg and GenTq=' num2str(SLin.GenTq/1e3) 'kNm (' simOpt.FSTnewModelSuffix ')...'])
            else
                disp(['> start SL sim with ' num2str(SLin.n_step) ' input steps (' simOpt.FSTnewModelSuffix ')...'])
            end
        end

        % run simulation (and suppress cmd window outputs)
        try
            if simOpt.disp>2
                simout=sim(simOpt.SLmodelName,simOpt.SLmaxTime,SLsimset);
            else
                [~,simout]=evalc('sim(simOpt.SLmodelName,simOpt.SLmaxTime,SLsimset)');
            end
            if ~ismember('outMax',get(simout))
                % if outMax is not provided, make a fake field
                simout.outMax=nan(size(simout.outAvg));
            end
            if ismember('outSnippet',get(simout))
                % store (cut) time series used for computing avg/max (only of very last parameter combination!)
                simRes.outSnippet=simout.outSnippet;
            end
        catch ex
            if strfind(ex.message,'Tower strike')
                n_done=SLin.n_step; % make the remaining simulatoin results nan
                simout.outAvg=nan(n_done,numel(simOpt.FSToutNameArr));
                simout.outMax=nan(n_done,numel(simOpt.FSToutNameArr));
                simout.isValid=false(n_done,1);
                simout.settlingTime=nan(n_done,1);
                warning(['tower strike in ' simOpt.FSTnewInputFile '; n_done=' num2str(n_done)])
            else
                rethrow(ex)
            end
        end
        simRes.outAvg=[simRes.outAvg;simout.outAvg];
        simRes.outMax=[simRes.outMax;simout.outMax];
        simRes.isValid=[simRes.isValid;simout.isValid];
        simRes.settlingTime=[simRes.settlingTime;simout.settlingTime];
            
        % check why not all input steps have been simulated
        n_done=size(simout.outAvg,1);
        if n_done~=SLin.n_step
            if strcmp(simout.SimulationMetadata.ExecutionInfo.StopEvent,'ReachedStopTime')
                 % current step has not been finished - redo step
                if simOpt.disp>1
                    disp(['> simulation has ended because max. simulation time SLMAXTIME is reached  (' simOpt.FSTnewModelSuffix ')'])
                end
                if n_done<0
                    error(['max. simulation time SLMAXTIME is too short for the signal to settle (' simOpt.FSTnewModelSuffix ')'])
                end
            elseif simout.settlingTime(end)>=simOpt.settlingMaxTime
                if simOpt.disp>1
                    disp(['> simulation has ended because of settling time out  (' simOpt.FSTnewModelSuffix ')'])
                end
            elseif ~simout.isValid(end) % assume that if settling time is not reached, but the results are invalid, then it must be due exceeding the limits
                if simOpt.disp>1
                    disp(['> simulation has ended because selected ouptut signals are outside of limits  (' simOpt.FSTnewModelSuffix ')'])
                end
            else
                if simOpt.disp>1
                    disp(['> simulation has ended because of user interruption  (' simOpt.FSTnewModelSuffix ')'])
                end
                break
            end
        end
        SLin.BldPitchC=SLin.BldPitchC(n_done+1:end);
        SLin.GenTq=SLin.GenTq(n_done+1:end);
        if isfield(SLin,'IPCgain')
            SLin.IPCgain=SLin.IPCgain(n_done+1:end);
        end
        if isfield(SLin, 'IPCoffset')
            SLin.IPCoffset=SLin.IPCoffset(n_done+1:end);
        end
        SLin.n_step=numel(SLin.BldPitchC);
    
    end

    % stop simulation and close model
    stopSimLOCAL()


catch ex

    % stop simulation and close model
    stopSimLOCAL()

    % rethrow error
    rethrow(ex)
    
end



%% nested function for stopping simulation
    function stopSimLOCAL()

        % close system if it was opened in the background (without GUI)
        if ~isSysLoaded
            close_system(simOpt.SLmodelName,0)
        end
        % delete variables from Sfunction
        evalin('base','clear DT OutList'); 
        % clear mex to avoid crash when starting the next time ('..memory already allocated..')
        clear mex %#ok<CLMEX> 
        % cleanup FAST output files (and suppress warning)
        w=warning('query','MATLAB:DELETE:FileNotFound');
        warning('off','MATLAB:DELETE:FileNotFound')
        delete([simOpt.FSTnewInputFile(1:end-4) '.SFunc.SD.ech']) 
        delete([simOpt.FSTnewInputFile(1:end-4) '.SFunc.SD.sum']) 
        delete([simOpt.FSTnewInputFile(1:end-4) '.SFunc.ED.sum']) 
        delete([simOpt.FSTnewInputFile(1:end-4) '.SFunc.outb'])
        delete([simOpt.FSTnewInputFile(1:end-4) '.SFunc.out'])
        delete([simOpt.FSTnewInputFile(1:end-4) '.SFunc.SrvD.Sum'])
        warning(w.state,'MATLAB:DELETE:FileNotFound')

    end

end