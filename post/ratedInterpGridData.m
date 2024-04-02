function [RtdWindSpeed,RtdBldPitch,RtdGenTq]=ratedInterpGridData(data_arr,RtdGenPwr,pVal_arr,varargin)
% note: units are as described in OpenFAST (i.e. GenPw is in kW)
% note: pVal_arr must be in the order of data_act.grid.dimNames(3:end)

% handle input
if numel(varargin)>=1
    fminconOpt=varargin{1};
else
    fminconOpt=[];
end

if numel(varargin)>=2
    interpOpt=varargin{2};
else
    interpOpt=[];
end


% reduce dimensions
% TODO!

% quick-find grid points where rated is crossed (Note: using advanced interpolation techniques this may not be exact)
isAboveRated=cellfun(@(x) max(x.GenPwr(:))>RtdGenPwr,data_arr);
i_br=find(diff(isAboveRated));
if numel(i_br)~=1
    error('rated wind speed could not be detected.')
end


%% get max. power for just below and just above rated

xName='BldPitchC';
yName='GenTq';

for ii=2:-1:1
    
    % re-assign variables
    data_act=data_arr{i_br+ii-1};
    windSpeed(ii)=data_act.windSpeed;
    % check if given parameters are on grid point
    if ~isequal(data_arr{1}.grid.dimNames(1:2),{xName yName})
        error('first two grid dimensions need to be BldPitchC and GenTq.')
    end
    % select/interpolate grid point
    [x,y,z,isValid]=getGridData2(data_act,'GenPwr',xName,yName,data_act.grid.dimNames(3:end),pVal_arr,false,interpOpt);
    % find maximum power
    [GenPwrMax(ii),xMax(ii),yMax(ii)]=maxInterp2(z,x,y,fminconOpt,interpOpt); % maximum of displayed surface

end


%% find rated wind speed using linear interpolation between 2 max. power points 
method='linear'; % only 2 points used!
RtdWindSpeed=interp1(GenPwrMax,windSpeed,RtdGenPwr,method);
if isnan(RtdWindSpeed)
    error('rated wind speed')
end
RtdBldPitch=interp1(windSpeed,xMax,RtdWindSpeed,method);
RtdGenTq=interp1(windSpeed,yMax,RtdWindSpeed,method);

%% find rated wind speed using linear interpolation at fixed BldPitch and GenTq (rated)
GenPwr1=interpGridData(data_arr{i_br},'GenPwr',[RtdBldPitch RtdGenTq pVal_arr],interpOpt);
GenPwr2=interpGridData(data_arr{i_br+1},'GenPwr',[RtdBldPitch RtdGenTq pVal_arr],interpOpt);
if RtdGenPwr>GenPwr2 || RtdGenPwr<GenPwr1
    warning('rBldPitch rGenTq interpolation failed. taking GenPwrMax interpolation.')
else
    RtdWindSpeed=interp1([GenPwr1 GenPwr2],windSpeed,RtdGenPwr,method);
end



