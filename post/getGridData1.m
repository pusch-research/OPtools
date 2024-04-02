function [x,y,isValid]=getGridData1(data,outName,xName,pDimName_arr,pVal_arr,isIdx,varargin)
% get 1D line from grid data
% TODO: handle isIdx=true

% input handling
n_grid=numel(data.grid.dimNames);
if nargin<3 || isempty(xName)
    i_xDim=1;
else
    i_xDim=findselection(xName,data.grid.dimNames);
end
if nargin<4 || isempty(pDimName_arr)
    rDim_iArr=setdiff(1:n_grid,[i_xDim]);
else
    rDim_iArr=findselection(pDimName_arr,data.grid.dimNames);
    if numel(rDim_iArr)~=n_grid-1
        error('wrong number of rName defined')
    end
end
if nargin<5 || isempty(pVal_arr)
    pVal_arr={':'};
elseif ~iscell(pVal_arr)
    pVal_arr=num2cell(pVal_arr);
end
if nargin<6
    isIdx=true;
else
    if isIdx==false
        error('not implemented.')
        % ToDo see getGridData2
    end
end

% x
x=data.grid.smplPts{i_xDim}(:);

% select y data and isValid
s=struct('type','()','subs',{[':' pVal_arr]});
y=permute(data.(outName),[i_xDim rDim_iArr]);
y=subsref(y,s);
y=y(:);
isValid=permute(data.isValid,[i_xDim rDim_iArr]);
isValid=subsref(isValid,s);


