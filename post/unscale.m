%% back-scale grid values
% see also scale.m
function val=unscale(valScaled,valMin,valRange)
% valScaled,valMin,valRange must be row vectors

if iscell(valScaled)
    val=cell(size(valScaled));
    for ii=1:numel(valScaled)
        val{ii}=unscale(valScaled{ii},valMin(ii),valRange(ii));
    end
else
    val=valMin+valScaled.*valRange;
end