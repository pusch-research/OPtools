function [schedule,rtd]=scheduleInterpGridData(data_arr,OutName_arr,pVal_arr,specs,ctrl,varargin)
% NOTE: schedule.GenTq is in [Nm]
% ToDo: handle constraints on inputs better/more general (e.g. PtfmPitch in/equality constraint...)
kNm2Nm=1e3;

%% input handling
if ~iscell(pVal_arr)
    pVal_arr=num2cell(pVal_arr);
end
if numel(varargin)>0
    fminconOpt=varargin{1};
else
    fminconOpt=[];
end
if numel(varargin)>1
    interpOpt=varargin{2};
    if ~iscell(interpOpt)
        interpOpt={interpOpt};
    end
else
    interpOpt={'makima',nan};
end
if ~iscell(OutName_arr)
    OutName_arr={OutName_arr};
end
if ~ismember('RotSpeed',OutName_arr)
    OutName_arr=[OutName_arr 'RotSpeed']; % RotSpeed always needs to be an output
end
if nargin<5 || isempty(ctrl)
    ctrl=struct();
end
if ~notempty(ctrl,'belowRtd') 
    ctrl.belowRtd=struct();
    ctrl.belowRtd.GenTq='opt'; % {'TSRtracking', 'Kw2', 'opt',struct('windSpeed',[],'GenTq',[])}
    ctrl.belowRtd.BldPitchC='opt'; % {'opt': to be optimized, 'finePitch': take specs.BldPitch_opt, scalar: given finePitch in [deg]}
end
if ~notempty(ctrl,'aboveRtd') 
    ctrl.aboveRtd=struct();
    ctrl.aboveRtd.GenTq='rtd'; % {'rtd': rated GenTq, scalar: given GenTq in [Nm]; 'opt': optimize using standard goal; 'optOutName': optimize output with name <OutName>}
    ctrl.aboveRtd.BldPitchC='opt'; % {'opt': optimize using standard goal (=maintain rated GenSpeed), 'optOutName': optimize ouput with name <OutName>}
    % NOTE: if GenTq is not 'rtd', there might be a transition phase
end
if strncmpi(ctrl.belowRtd.BldPitchC,'fine',4)
    ctrl.belowRtd.BldPitchC=specs.BldPitch_opt;
end
% constraints (NOTE: are applied on outputs, i.e., GenTq is in [kNm])
if ~notempty(ctrl.belowRtd,'cnstr')
    ctrl.belowRtd.cnstr=struct('OutName',{{}},'lb',[],'ub',[]); 
end
if ~notempty(ctrl.aboveRtd,'cnstr')
    ctrl.aboveRtd.cnstr=struct('OutName',{{}},'lb',[],'ub',[]); % Note: if a value is set to 'nan', the rated value is taken!
end

% checks
if strncmpi(ctrl.belowRtd.BldPitchC,'opt',3) && strncmpi(ctrl.belowRtd.GenTq,'opt',3) && ...
        ~strcmpi(ctrl.belowRtd.BldPitchC,ctrl.belowRtd.GenTq)
    error('When optimizing BldPitchC AND GenTq in below rated conditions, the given outNames must be identical.') % not implemented anyway
end
if strncmpi(ctrl.aboveRtd.BldPitchC,'opt',3) && strncmpi(ctrl.aboveRtd.GenTq,'opt',3) && ...
        ~strcmpi(ctrl.aboveRtd.BldPitchC,ctrl.aboveRtd.GenTq)
    error('When optimizing BldPitchC AND GenTq in above rated conditions, the given outNames must be identical.')
end


%% init
n_wind=numel(data_arr);
n_out=numel(OutName_arr);
n_dim=numel(data_arr{1}.grid.dimNames);
windSpeed_arr=cellfun(@(x) x.windSpeed,data_arr);
schedule=struct(); % schedule for BldPitch, GenTq, and requested outputs
rtd=struct(); % rated values
i_dimBldPitchC=data_arr{1}.grid.dimIdx.BldPitchC;
if i_dimBldPitchC~=1
    error('not implemented.')
end
i_dimGenTq=data_arr{1}.grid.dimIdx.GenTq;
if i_dimGenTq~=2
    error('not implemented.')
end
isAboveRated=false;




%% loop wind speeds
for i_wind=1:n_wind
    
    % re-assign variables
    data_act=data_arr{i_wind};
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% BELWO RATED (incl. rated)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isAboveRated

        % select BldPitchC
        if isnumeric(ctrl.belowRtd.BldPitchC)
            % BldPitchC is given
            tmpBldPitchC=ctrl.belowRtd.BldPitchC;
        elseif isstruct(ctrl.belowRtd.BldPitchC)
            % schedule given - interpolate
            tmpBldPitchC=interp1([ctrl.belowRtd.BldPitchC.windSpeed],...
                                 [ctrl.belowRtd.BldPitchC.BldPitchC],...
                                 data_act.windSpeed,interpOpt{:});
        elseif strncmpi(ctrl.belowRtd.BldPitchC,'opt',3)
            % BldPitchC is optimized
            tmpBldPitchC=data_act.grid.BldPitchC;
        else
            error('not implemented.')
        end

        
        if isstruct(ctrl.belowRtd.GenTq)
            %% GenTq schedule given
            tmpGenTq=interp1(ctrl.belowRtd.GenTq.windSpeed,ctrl.belowRtd.GenTq.GenTq,data_act.windSpeed,interpOpt{:});
            x_act=[tmpBldPitchC tmpGenTq pVal_arr];
            [GenPwr_act,xMax]=maxInterpGridData(data_act,'GenPwr',fminconOpt,interpOpt,x_act,ctrl.belowRtd.cnstr);
            x_act=num2cell(xMax);
        elseif  strncmpi(ctrl.belowRtd.GenTq,'opt',3) 
            %% GenTq optimized
            % find maximum power for all (selected) BldPitchC and GenTq
            x_act=[tmpBldPitchC data_act.grid.GenTq pVal_arr];
            [GenPwr_act,xMax]=maxInterpGridData(data_act,'GenPwr',fminconOpt,interpOpt,x_act,ctrl.belowRtd.cnstr);
            x_act=num2cell(xMax);
        else 
            %% GenTq ctrl law (Kw2 or TSR tracking)
            if strcmpi(ctrl.belowRtd.GenTq,'TSRtracking')
                % tip speed ratio tracking controller (fix rotor radius assumed)
                RotSpeed_opt_rpm=specs.TSR_opt*data_act.windSpeed/specs.R*radDs2rpm; % optimal RotSpeed at this WindSpeed (assumes fix R)
                data_act.BelowRtdCtrlDiff=data_act.RotSpeed-RotSpeed_opt_rpm; % =0 to fullfill the TSR tracking control law 
            elseif strcmpi(ctrl.belowRtd.GenTq,'Kw2')
                % Kw2 controller
                K=1/2*specs.rho*pi*specs.R^5*specs.Cp_opt/(specs.TSR_opt*specs.GBRatio)^3; % K value from Kw2 law
                data_act.BelowRtdCtrlDiff=data_act.GenTq*kNm2Nm-K*rpm2radDs(data_act.GenSpeed).^2; % =0 to fullfill the Kw2 control law 
            else
                error('not implemented.')
            end
            data_act.BelowRtdCtrlDiff(~data_act.isValid)=nan; % set invalid values to nan (maybe skip?)

            % handle GenTq contstraint here (since it is an input)
            tmpGenTq=data_act.grid.GenTq;
            i_cnstrGenTq=find(ismember(ctrl.belowRtd.cnstr.OutName,'GenTq'));
            if ~isempty(i_cnstrGenTq)
                % Inconsistency: OpenFAST GenTq input is in [Nm] and output is in [kNm]
                % constraints are given for output side, x_act is given for input side (hence conversion is needed!)   
                tmpGenTq(tmpGenTq<ctrl.belowRtd.lb(i_cnstrGenTq)/kNm2Nm || ...
                    tmpGenTq>ctrl.belowRtd.ub(i_cnstrGenTq)/kNm2Nm )=[];
                if isempty(tmpGenTq)
                    error('GenTq is out of range. Change constraint.')
                end
                if ~isinf(ctrl.belowRtd.lb(i_cnstrGenTq))
                    tmpGenTq=[ctrl.belowRtd.lb(i_cnstrGenTq) tmpGenTq]; %#ok<AGROW> 
                end
                if ~isinf(ctrl.belowRtd.ub(i_cnstrGenTq))
                    tmpGenTq=[tmpGenTq ctrl.belowRtd.lb(i_cnstrGenTq)]; %#ok<AGROW> 
                end
            end

            x_act=[tmpBldPitchC tmpGenTq pVal_arr];
            if all(cellfun(@numel,pVal_arr)==1) && numel(tmpBldPitchC)==1 
                %% no optimization needed 
                % solve 1D equality constraint - ToDo: move this to maxInterpGridData.m
                tmpCtrl=squeeze(interpGridData(data_act,'BelowRtdCtrlDiff',x_act,interpOpt{:}));
                if ~isvector(tmpCtrl)
                    error('not implemented')
                elseif ~(any(tmpCtrl>0) && any(tmpCtrl<0))
                    error('below rated control law cannot be fullfilled for selected BldPitchC.')
                end
                x_act{i_dimGenTq}=interp1(tmpCtrl(~isnan(tmpCtrl)),data_act.grid.GenTq(~isnan(tmpCtrl)),0,interpOpt{:});
            else
                %% optmize over free parameters (e.g. BldPitchC)    
                % add equality constraint
                tmpCnstr.OutName=[ctrl.belowRtd.cnstr.OutName 'BelowRtdCtrlDiff'];
                tmpCnstr.lb=[ctrl.belowRtd.cnstr.lb 0];
                tmpCnstr.ub=[ctrl.belowRtd.cnstr.ub 0];
                % run optimization
                [~,xMax]=maxInterpGridData(data_act,'GenPwr',fminconOpt,interpOpt,x_act,tmpCnstr);
                x_act=num2cell(xMax);
            end

            % check
            if x_act{i_dimGenTq}>max(tmpGenTq) || x_act{i_dimGenTq}<min(tmpGenTq)
                warning(['GenTq=' num2str(x_act{i_dimGenTq}) ' is outside of (valid) gridding values (i_wind=' num2str(i_wind) ', ' num2str(data_act.windSpeed) ' m/s)'])
            end
            GenPwr_act=interpGridData(data_act,'GenPwr',x_act,interpOpt{:});
            
        end

        % check
        checkValid=interpGridData(data_act,'isValid',x_act,'linear',nan);
        if isnan(checkValid) || checkValid==0
            error('found optimium is not valid. Consider suppressing extrapolation or set invalid values differently.')
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% RATED (check if just crossed from below to above rated)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        if GenPwr_act>specs.rGenPwr % in kW (OpenFast unit)

            % check
            if i_wind==1
                error('GenPwr at lowest wind speed is already above rated GenPwr.')
            end

            % find rated wind speed using linear interpolation between 2 GenPower points 
            tmpWindSpeed12=windSpeed_arr(i_wind-1:i_wind);
            tmpGenPwr12=[schedule(i_wind-1).GenPwr GenPwr_act];
            rtd.WindSpeed=interp1(tmpGenPwr12,tmpWindSpeed12,specs.rGenPwr);
            x_rtd=cell(1,n_dim);
            for i_dim=1:n_dim
                dimName_act=data_act.grid.dimNames{i_dim};
                tmpDimVal12=[schedule(i_wind-1).(dimName_act) x_act{i_dim}];
                rtd.(dimName_act)=interp1(tmpWindSpeed12,tmpDimVal12,rtd.WindSpeed);
                x_rtd{i_dim}=rtd.(dimName_act);
            end
            
            % update rated wind speed using linear interpolation at fixed BldPitch and GenTq (both rated)
            tmpGenPwr12=[interpGridData(data_arr{i_wind-1},'GenPwr',x_rtd,interpOpt{:})...
                         interpGridData(data_arr{i_wind},'GenPwr',x_rtd,interpOpt{:})]; 
            if any(isnan(tmpGenPwr12))
                error('Interpolation at rated BldPitchC/GenTq did not succeed. Change constraints or control law parameters (e.g. cp_opt or TSR_opt defined in specs).')
            end
            if specs.rGenPwr>tmpGenPwr12(2) || specs.rGenPwr<tmpGenPwr12(1)
                warning('RtdBldPitch-RtdGenTq interpolation failed. Doing GenPwr interpolation.')
                tmpGenPwr12=[schedule(i_wind-1).GenPwr GenPwr_act];
            else
                rtd.WindSpeed=interp1(tmpGenPwr12,tmpWindSpeed12,specs.rGenPwr);
            end

            % update rated values
            rtd.GenPwr=specs.rGenPwr; % might be overwritten
            for i_out=1:n_out
                % compute requested output values (by interpolation at RtdBldPitch and RtdGenTq)
                outName_act=OutName_arr{i_out};
                tmpOut12=[interpGridData(data_arr{i_wind-1},outName_act,x_rtd,interpOpt{:})...
                          interpGridData(data_arr{i_wind},outName_act,x_rtd,interpOpt{:})];
                rtd.(outName_act)=interp1(tmpGenPwr12,tmpOut12,specs.rGenPwr,'linear');
            end

            % overwrite act values to be stored
            GenPwr_act=rtd.GenPwr;
            x_act{i_dimBldPitchC}=rtd.BldPitchC;
            x_act{i_dimGenTq}=rtd.GenTq;

            % overwrite 'nan' values in aboveRtd.cnstr with rated values
            for i_cnstr=1:numel(ctrl.aboveRtd.cnstr.OutName)
                if isnan(ctrl.aboveRtd.cnstr.ub(i_cnstr))
                    ctrl.aboveRtd.cnstr.ub(i_cnstr)=rtd.(ctrl.aboveRtd.cnstr.OutName{i_cnstr});
                end
                if isnan(ctrl.aboveRtd.cnstr.lb(i_cnstr))
                    ctrl.aboveRtd.cnstr.lb(i_cnstr)=rtd.(ctrl.aboveRtd.cnstr.OutName{i_cnstr});
                end
            end
            
            % set flag and store rated values
            isAboveRated=true;
            schedule(i_wind)=rtd; % additional data point exactly when rated wind speed is reached

        end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ABOVE RATED
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isAboveRated
        
        % above rated
        GenPwr_act=specs.rGenPwr;
        x_act=[data_act.grid.BldPitchC rtd.GenTq pVal_arr]; 

        % GenTq
        if strcmpi(ctrl.aboveRtd.GenTq,'rtd')
            % set GenTq to (internally determined) rated value
            x_act{i_dimGenTq}=rtd.GenTq;
            aboveRtdRotSpeed=specs.rGenPwr*1e3/x_act{i_dimGenTq}*radDs2rpm*specs.GBRatio;
        elseif isnumeric(ctrl.aboveRtd.GenTq)
            % set to given value (leads to jumps - not recommended)
            x_act{i_dimGenTq}=ctrl.aboveRtd.GenTq;
            aboveRtdRotSpeed=specs.rGenPwr*1e3/x_act{i_dimGenTq}*radDs2rpm*specs.GBRatio;
        elseif strncmpi(ctrl.aboveRtd.GenTq,'opt',3)
            % optimize given outName 
            x_act{i_dimGenTq}=data_act.grid.GenTq; % full grid vector
            aboveRtdRotSpeed=[];
        else
            error('not implemented.')
        end


        % BldPitchC
        if strncmpi(ctrl.aboveRtd.BldPitchC,'opt',3)  % first three letters are 'opt'
            %% BldPitchC determination
            n_SmplPts=cellfun(@numel,x_act);
            isNDproblem=max(n_SmplPts)~=prod(n_SmplPts);
            isOPTproblem=length(ctrl.aboveRtd.BldPitchC)>3;
            if isNDproblem ||  isOPTproblem
                %% general optimization
                tmpCnstr=ctrl.aboveRtd.cnstr; %struct('OutName',{['GenPwr' cnstr.OutName]},'lb',[specs.rGenPwr cnstr.lb],'ub',[specs.rGenPwr cnstr.ub]); 
                if isOPTproblem
                    % maximize a given outName, e.g., (negative) loads, (negative) distance to reference GenSpeed,..
                    objOutName=ctrl.aboveRtd.BldPitchC(4:end); % outName to be maximized
                    tmpCnstr=struct('OutName',{['GenPwr' tmpCnstr.OutName]},'lb',[specs.rGenPwr tmpCnstr.lb],'ub',[specs.rGenPwr tmpCnstr.ub]);  % add rated GenPwr equality constraint
                else
                    % maximize (negative) distance of GenPwr to RtdGenPwr (reformulate equality constraint as objective)
                    if ~any(tmpCnstr.ub==tmpCnstr.lb)
                        error('Not solvable. Maybe reduce dimensions by fixing some values, define an optimization function or equality constraints.')
                    end
                    data_act.GenPwrDist=-abs(data_act.GenPwr-specs.rGenPwr);
                    objOutName='GenPwrDist';
                end               
                % find optimum
                [~,xMax]=maxInterpGridData(data_act,objOutName,fminconOpt,interpOpt,x_act,tmpCnstr);
                x_act=num2cell(xMax);
            else
                %% Special Case: 1D problem - find BldPitchC such that RtdRotSpeed is achived
                % ToDo: instead of RtdRotSpeed go for RtdGenPwr
                tmpRotSpeed_arr=interpGridData(data_act,'RotSpeed',x_act,interpOpt{:});
                n_crossings=sum(abs(diff(tmpRotSpeed_arr>aboveRtdRotSpeed)));
                if n_crossings==1 || n_crossings==2 
                    % HACK I: select only the part after the peak of RotSpeed (assuming that RotSpeed decreases with BldPitchC); IMPORTANT for 'spline' interpolation
                    % HACK II: in case RotSpeed is identical at last points, use the one at lower BldPitchC
                    [~,i_TmpRotSpeedMax]=max(tmpRotSpeed_arr);
                    sel_iArr=intersect(i_TmpRotSpeedMax:numel(tmpRotSpeed_arr),find(~isnan(tmpRotSpeed_arr)));
                    while tmpRotSpeed_arr(sel_iArr(end))==tmpRotSpeed_arr(sel_iArr(end-1))
                        sel_iArr(end)=[]; 
                    end
                    x_act{i_dimBldPitchC}=interp1(tmpRotSpeed_arr(sel_iArr),data_act.grid.BldPitchC(sel_iArr),aboveRtdRotSpeed,interpOpt{:});
                elseif n_crossings<1
                    % given GenTq does not allow operation at RtdGenPwr -> change GenTq
                    % 1) find rated GenPwr contour
                    [~,~,tmpGenPwr,isValid]=getGridData2(data_act,'GenPwr','BldPitchC','GenTq',data_act.grid.dimNames(3:end),pVal_arr,false);
                    tmpGenPwr(~isValid)=nan;
                    rtdGenPwrContour=contourc(data_act.grid.BldPitchC,data_act.grid.GenTq,tmpGenPwr,[0 specs.rGenPwr]);
                    minGenTqDiff=inf;
                    % 2) evaluate all contour parts and find valid GenTq closest to given one
                    while size(rtdGenPwrContour,2)>0
                        if rtdGenPwrContour(1,1)==specs.rGenPwr % rated power contour line
                            warning('check if coordinates are exchanged! contour!')
                            GenTq_arr=rtdGenPwrContour(2,2:rtdGenPwrContour(2,1)+1);
                            [tmpMin,i_min]=min(abs(GenTq_arr-x_act{i_dimGenTq}));
                            if tmpMin<abs(minGenTqDiff)
                                minGenTqDiff=GenTq_arr(i_min)-x_act{i_dimGenTq};
                            end
                        end
                        rtdGenPwrContour(:,1:rtdGenPwrContour(2,1)+1)=[];
                    end
                    x_act{i_dimGenTq}=x_act{i_dimGenTq}+minGenTqDiff; % update GenTq
                    % 3) re-do BldPitch optimization (incl. HACKS from above) assuming n_crossings==1 || n_crossings==2 now 
                    tmpRotSpeed_arr=interpGridData(data_act,'RotSpeed',x_act,interpOpt{:});
                    [~,i_TmpRotSpeedMax]=max(tmpRotSpeed_arr);
                    sel_iArr=intersect(i_TmpRotSpeedMax:numel(tmpRotSpeed_arr),find(~isnan(tmpRotSpeed_arr)));
                    while tmpRotSpeed_arr(sel_iArr(end))==tmpRotSpeed_arr(sel_iArr(end-1))
                        sel_iArr(end)=[]; 
                    end
                    x_act{i_dimBldPitchC}=interp1(tmpRotSpeed_arr(sel_iArr),data_act.grid.BldPitchC(sel_iArr),rtd.RotSpeed,interpOpt{:});
%                     error(['Rated RotSpeed is outside of given RotSpeed range @ i_wind=' num2str(i_wind) ', ' num2str(data_act.windSpeed) ' m/s.']')
                elseif n_crossings>2
                    error(['Multiple crossings of RotSpeed with rated RotSpeed. No unique solution can be found @ i_wind=' num2str(i_wind) ', ' num2str(data_act.windSpeed) ' m/s.']')
                end
            end
        else
            error('not implemented.')
        end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% STORE VALUES
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ii=i_wind+isAboveRated; % note: when rated wind speed is crossed, the 'rated' data point is added and array is extended by 1 wind speed
    schedule(ii).WindSpeed=windSpeed_arr(i_wind);
    schedule(ii).GenPwr=GenPwr_act;
    for i_dim=1:n_dim % note: may be overwritten if dimName is part of outName_arr
        dimName_act=data_act.grid.dimNames{i_dim};
        schedule(ii).(dimName_act)=x_act{i_dim};
    end
    for i_out=1:n_out
        outName_act=OutName_arr{i_out};
        schedule(ii).(outName_act)=interpGridData(data_act,outName_act,x_act,interpOpt{:});
    end

end


if ~isAboveRated
    warning('RatedNotReached','rated generator power not reached.');
end

