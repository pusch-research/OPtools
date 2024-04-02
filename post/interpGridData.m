function outVal=interpGridData(data,outName,dimVal_arr,varargin)


% input handling
interpOpt=varargin; 
if numel(interpOpt)<1
    interpOpt{1}='linear';
end
if numel(interpOpt)<2
    interpOpt{2}=nan; % set extrapval=nan to suppress extrapolation; PROBLEM with makima/spline sometimes (when nan values are nearby)
end
if ~iscell(dimVal_arr)
    dimVal_arr=num2cell(dimVal_arr);
end
if any(cellfun(@(x) any(isnan(x(:))),dimVal_arr))
    error('interpolation point is nan.')
end

% dimension reduction if necessary (do this outside of this function for faster calculations)
nsDim_iArr=cellfun(@(x) numel(x)>1,data.grid.dimSmplPts); % non-singular dimension
v=squeeze(data.(outName));
if islogical(v)
    v=double(v); % convert
end
v(~data.isValid)=nan; % make all non-valid values to nan - problem for makima/spline interpolation method
dimVal_arr=dimVal_arr(nsDim_iArr);
x=data.grid.dimSmplPts(nsDim_iArr);

% reshape dimVal_arr (assume dimVal_arr are grid vectors)
if numel(dimVal_arr)>1
    % make sure that first two dimensions have a different orientation!
    % THIS IS A MATLAB BUG in C:\Program Files\MATLAB\R2021b\toolbox\matlab\polyfun\private\compactgridformat.m
    dimVal_arr{1}=dimVal_arr{1}(:);
    dimVal_arr{2}=dimVal_arr{2}(:)';
end
% n_dim=numel(dimVal_arr);
% for i_dim=1:n_dim
%     sz=ones(1,n_dim);
%     sz(i_dim)=numel(dimVal_arr{i_dim});
%     dimVal_arr{i_dim}=reshape(dimVal_arr{i_dim},sz);
% end

% interpolate
outVal=interpn(x{:},v,dimVal_arr{:},interpOpt{:});