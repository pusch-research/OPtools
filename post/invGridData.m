function x=invGridData(data,outName,outValue,xName,varargin)

% compute x value for a given outValue using linear interpolation
[x_arr,v_arr]=getGridData1(data,outName,xName,varargin{:});
x=intersections(x_arr,v_arr,minmax(x_arr'),[outValue outValue]); % linear interpolation!
