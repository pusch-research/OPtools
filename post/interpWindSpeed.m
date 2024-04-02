function outVal_arr=interpWindSpeed(data_arr,outName_arr,windSpeed,dimVal_arr,varargin)
% interpolate outputs between windspeeds

% input handling
if ~iscell(outName_arr)
    outName_arr={outName_arr};
end

% quick-find windSpeed grid points around given windSpeed
i1=find(diff(cellfun(@(x) x.windSpeed>windSpeed,data_arr)));
if numel(i1)~=1
    error('given wind speed is outside of grid.')
end
i2=i1+1;
windSpeed_arr=[data_arr{i1}.windSpeed data_arr{i2}.windSpeed];

% 
for i_out=numel(outName_arr):-1:1
    outVal1=interpGridData(data_arr{i1},outName_arr{i_out},dimVal_arr,varargin{:});
    outVal2=interpGridData(data_arr{i2},outName_arr{i_out},dimVal_arr,varargin{:});
    outVal_arr(i_out)=interp1(windSpeed_arr,[outVal1 outVal2],windSpeed,varargin{:});
end


