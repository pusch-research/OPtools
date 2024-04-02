function [x,y,z,isValid]=getGridData2(data,outName,xName,yName,pDimName_arr,pVal_arr,isIdx,varargin)
% get 2D slice of grid data

% input handling
n_grid=numel(data.grid.dimNames);
if nargin<3 || isempty(xName)
    i_xDim=1;
else
    i_xDim=findselection(xName,data.grid.dimNames);
end
if nargin<4 || isempty(yName)
    i_yDim=2;
else
    i_yDim=findselection(yName,data.grid.dimNames);
end
if nargin<5 || isempty(pDimName_arr)
    pDim_iArr=setdiff(1:n_grid,[i_xDim i_yDim]);
else
    pDim_iArr=findselection(pDimName_arr,data.grid.dimNames);
    if numel(pDim_iArr)~=n_grid-2
        error('wrong number of subName defined')
    end
end
if nargin<6 || isempty(pVal_arr)
    pVal_arr={':'};
elseif ~iscell(pVal_arr)
    pVal_arr=num2cell(pVal_arr);
end
if nargin<7 || isempty(isIdx)
    isIdx=true; % default: assume rVal_arr are indices!
end


if ~isIdx

    % check if given rVal are on grid point (and find indices of that grid point)
    n_p=numel(pDim_iArr);
    pVal_iArr=nan(n_p,1);
    for i_r=1:n_p
        i_tmp=find(data.grid.dimSmplPts{pDim_iArr(i_r)}==pVal_arr{i_r});
        if length(i_tmp)==1
            pVal_iArr(i_r)=i_tmp;
        end
    end

    % get data
    if all(~isnan(pVal_iArr))

        % fixed parameters are exactly at a grid point - select grid point
        isIdx=true;
        pVal_arr=num2cell(pVal_iArr);
        % continue below..

    else

        % perform interpolation if one or more parameters are not on a grid point
        gridSmplPts_act=data.grid.dimSmplPts;
        gridSmplPts_act(pDim_iArr)=pVal_arr;
        [x,y]=ndgrid(data.grid.(xName),data.grid.(yName));
        z=squeeze(interpGridData(data,outName,gridSmplPts_act,varargin{:}));
        isValid=squeeze(interpGridData(data,'isValid',gridSmplPts_act))==1;
        if i_xDim>i_yDim
            % permute first and second dimensions
            z=z.';
            isValid=isValid.';
        end

    end
end


if isIdx

    % generate x-y grid
    [x,y]=ndgrid(data.grid.dimSmplPts{i_xDim},data.grid.dimSmplPts{i_yDim});
    
    % select z data and isValid
    s=struct();
    s.type='()';
    s.subs=[':' ':' pVal_arr(:)'];
    z=permute(data.(outName),[i_xDim i_yDim pDim_iArr]);
    z=subsref(z,s);
    isValid=permute(data.isValid,[i_xDim i_yDim pDim_iArr]);
    isValid=subsref(isValid,s);

end


