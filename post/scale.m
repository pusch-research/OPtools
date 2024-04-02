%% scale values for optimization
% see also unscale.m
function [valScaled,valMin,valRange]=scale(val,fieldName_arr)

if iscell(val)
    % value cell array - loop thorugh array and return (cell) arrays
    valScaled=cell(size(val));
    valMin=nan(size(val));
    valRange=nan(size(val));
    for ii=1:numel(val)
        [valScaled{ii},valMin(ii),valRange(ii)]=scale(val{ii});
    end
elseif isstruct(val)
    % value structure - scale selected fieldnames and return (cell) arrays
    valScaled=cell(size(fieldName_arr));
    valMin=nan(size(fieldName_arr));
    valRange=nan(size(fieldName_arr));
    if ~iscell(fieldName_arr)
        fieldName_arr={fieldName_arr};
    end
    for ii=1:numel(fieldName_arr)
        [valScaled{ii},valMin(ii),valRange(ii)]=scale(val.(fieldName_arr{ii}));
    end
else
    % value numeric array - scale and return scalars
    valMin=min(val(:));
    valRange=range(val(:));
    if valRange<sqrt(eps)
        valRange=1;
    end
    valScaled=(val-valMin)/valRange;
end