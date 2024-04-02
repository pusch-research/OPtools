function [selData,keepDim_iArr]=selectGridData(data,dimVal_arr,outName_arr,varargin)
% select subset of grid data preserving the data struct (for more efficient interpolation/optimization)

% NOTE: invalid values may be completely out of range so it might be better to put them at least inside the valid range

% re-assign
interpOpt=varargin;
n_dim=numel(data.grid.dimNames);

% data array (for different wind speeds) is given - evaluate each element separately and return array
if iscell(data)
    for i_wind=numel(data):-1:1
        [selData{i_wind},keepDim_iArr]=selectGridData(data{i_wind},dimVal_arr,outName_arr,interpOpt{:});
    end
    return
end



% input handling
reservedFieldName_arr={'windSpeed','grid'};
if nargin<3 || isempty(outName_arr)
    outName_arr=setdiff(fieldnames(data),reservedFieldName_arr);
elseif ~iscell(outName_arr)
    outName_arr={outName_arr};
end
for i_origDim=1:n_dim
    if isempty(dimVal_arr{i_origDim}) || isequal(dimVal_arr{i_origDim},':')
        % select all values
        dimVal_arr{i_origDim}=data.grid.dimSmplPts{i_origDim};
    end
end

% exactly grid points are given - select them first (before interpolation)
selector=struct('type','()','subs',{cell(n_dim,1)});
isAllGridPoint=true;
for i_dim=1:numel(data.grid.dimSmplPts)
    if ~all(ismember(dimVal_arr{i_dim},data.grid.dimSmplPts{i_dim}))
        isAllGridPoint=false;
        selector.subs{i_dim}=':';
    else
        selector.subs{i_dim}=ismember(data.grid.dimSmplPts{i_dim},dimVal_arr{i_dim});
        dimName_act=data.grid.dimNames{i_dim};
        data.grid.(dimName_act)=data.grid.dimSmplPts{i_dim}(selector.subs{i_dim});
        data.grid.dimSmplPts{i_dim}=data.grid.dimSmplPts{i_dim}(selector.subs{i_dim});
    end
end
data.isValid=subsref(data.isValid,selector);

% selected grid
keepDim_iArr=cellfun(@(x) numel(x)>1,dimVal_arr); % indices of dimensions to be deleted
if ~any(keepDim_iArr)
    warning('single point is selected. dim-fields are not set.')
end
selData=struct();
selData.windSpeed=data.windSpeed;
selData.grid.dimNames=data.grid.dimNames(keepDim_iArr);
selData.grid.dimUnits=data.grid.dimUnits(keepDim_iArr);
for i_selDim=1:numel(selData.grid.dimNames)
    dimName_act=selData.grid.dimNames{i_selDim};
    i_origDim=findselection(dimName_act,data.grid.dimNames);
    selData.grid.dimIdx.(dimName_act)=i_selDim;
    selData.grid.dimSmplPts{i_selDim}=dimVal_arr{i_origDim};
    selData.grid.(dimName_act)=selData.grid.dimSmplPts{i_selDim};
end


% loop outputs
for i_out=1:numel(outName_arr)
    outName_act=outName_arr{i_out};
    % pre-select
    data.(outName_act)=subsref(data.(outName_act),selector);
    % interpolate (if required)
    if ~isAllGridPoint
        selData.(outName_act)=squeeze(interpGridData(data,outName_act,dimVal_arr,interpOpt{:}));
    else
        selData.(outName_act)=squeeze(data.(outName_act));
    end
end

% save isValid if not interpolation is done
if isAllGridPoint
    selData.isValid=squeeze(data.isValid);
end
