function [outMax,xMax,info]=maxInterpGridData(data,objOutName,fminconOpt,interpOpt,dimSelVal,cnstr)
% find maximum in grid data using interpolation
% TODO: use griddedInterpolant instead of interpn?


% handle input
if nargin<3 || isempty(fminconOpt)
    fminconOpt=optimoptions('fmincon','Display','none');
end
if nargin<4 || isempty(interpOpt)
    interpOpt={'makima'};
elseif ~iscell(interpOpt)
    interpOpt={interpOpt};
end
if nargin<5 || isempty(dimSelVal)
    dimSelVal=data.grid.dimSmplPts; % select all data points
end
if nargin<6 || isempty(cnstr)
    cnstr=struct();
    cnstr.ub=[];
    cnstr.lb=[];
    cnstr.OutName={};
end

% init
allOutName_arr=[{objOutName} cnstr.OutName];
[data,keepDim_iArr]=selectGridData(data,dimSelVal,allOutName_arr,interpOpt{:}); % reduce dimensions for faster computation times
n_x=numel(data.grid.dimNames);
if isfield(data,'isValid')
    if any(data.isValid(:))
        for i_out=1:numel(allOutName_arr)
            data.(allOutName_arr{i_out})(~data.isValid)=nan;
        end
    else
        % IMPORTANT: invalid values may used here which can lead to problems / inconsistent results (interpGridData.m does not consider invalid values)
        % SOLUTION: replace invalid values outside this function using extrapolation (recommended)
        warning('maxInterpGridData.m: invalid values are possibly used for solving the optimization problem. make sure that invalid values are within range.')
    end
end


% scale
[xDataScaled,xMin,xRange]=scale(data.grid.dimSmplPts); % sampling grid
[objDataScaled,objMin,objRange]=scale(data.(objOutName)); % output values used in objective function
[cnstrDataScaled,cnstrMin,cnstrRange]=scale(data,cnstr.OutName); % output values used in constraints
cnstr.lb=(cnstr.lb-cnstrMin)./cnstrRange; % scale lower bound
cnstr.ub=(cnstr.ub-cnstrMin)./cnstrRange; % scale upper bound

% solve optimization problem
isSpecialCase=sum(cnstr.lb==cnstr.ub)==1 && numel(xDataScaled)==2;
if isSpecialCase
    %% special case: 2D problem with single equality constraint
    % reduce number of optimization variables from 2 to 1


    % get equality constraint
    i_eqCnstr=cnstr.lb==cnstr.ub;
    % get data for finding contour (fine equidistant gridding!)
    [x1q,x2q]=meshgrid(linspace(0,1,100)); % same sampling in both directions
    cntrDataScaled=reshape(interpn(xDataScaled{:},cnstrDataScaled{i_eqCnstr},x1q(:),x2q(:),interpOpt{:}),size(x1q))';
    % find contour
    zeroScaled=cnstr.lb(i_eqCnstr);
    cntr=contourc(x1q(1,:),x2q(:,1),cntrDataScaled,[zeroScaled 2]); % note: min. 2 countours need to be evaluated (here 2 is chosen since it is for sure outside of range)
    % evaluate all contour parts
    xq_cntr={};
    while size(cntr,2)>0
        if cntr(1,1)==cnstr.lb(i_eqCnstr) % contour==0 line
            % IMPORTANT: contourc swaps dimensions! (x->GenTq,y->BldPitchC)
            xq_cntr{end+1}(1,:)=cntr(2,2:cntr(2,1)+1); %#ok<AGROW>  BldPitchC
            xq_cntr{end}(2,:)=cntr(1,2:cntr(2,1)+1); %#ok<AGROW>    GenTq
        end    
        cntr(:,1:cntr(2,1)+1)=[];
    end
    if numel(xq_cntr)~=1
        error(['number of contour segmenents is ' num2str(numel(xq_cntr))])
    end
    xq_cntr=[xq_cntr{:}]; % [2 x n_xp] 
    % compute p coordinate
    n_xp=size(xq_cntr,2);
    xp_cntr=zeros(1,n_xp); % [1 x n_xp]
    for i_xp=2:n_xp
       xp_cntr(i_xp)=xp_cntr(i_xp-1)+norm(xq_cntr(:,i_xp)); % euklid norm for distance between points
    end
    % coordinate transoformation
    xp2xq=@(xp) interp1(xp_cntr,xq_cntr',xp,'linear'); % returns row vector [1 x 2]
    % remove equality constraint
    cnstr.lb(i_eqCnstr)=[];
    cnstr.ub(i_eqCnstr)=[];
    cnstr.OutName(i_eqCnstr)=[];
    cnstrDataScaled=cnstrDataScaled(~i_eqCnstr);
    % compute objData and cnstrData on contour
    % IMPORTANT: data needs to be transposed - see interp2.m (THIS IS NOT INTUITIVE ARGH!)
    objDataScaled_cntr=interp2(xDataScaled{:},objDataScaled.',xq_cntr(1,:),xq_cntr(2,:),interpOpt{:});
    cnstrDataScaled_cntr=cell(numel(cnstr.OutName),1);
    for i_cnstr=numel(cnstr.OutName):-1:1
        cnstrDataScaled_cntr{i_cnstr}=interp2(xDataScaled{:},cnstrDataScaled{i_cnstr}.',xq_cntr(1,:),xq_cntr(2,:),interpOpt{:});
    end
    % initial value
    [~,xp0]=getMaxFeasGrdPntLOCAL(objDataScaled_cntr,{xp_cntr},cnstrDataScaled_cntr,cnstr.lb,cnstr.ub);
    % xp upper bound
    xpUpper=max(xp_cntr);

else
    %% regular case
   
    % coordinate transoformation
    xp2xq=@(xp) xp;
    % initial value
    [~,xp0]=getMaxFeasGrdPntLOCAL(objDataScaled,xDataScaled,cnstrDataScaled,cnstr.lb,cnstr.ub);
    % xp upper bound
    xpUpper=cellfun(@(x) double(range(x)>0),xDataScaled(:)); % make upper bound 0 in case range is zero, else one

end
    

% objective and constraint function
fObj=@(xp) -interpnLOCAL(xDataScaled,objDataScaled,xp2xq(xp),interpOpt{:}); % negative for maximization
fCnstr=@(xp) calcConstrLOCAL(xDataScaled,cnstrDataScaled,xp2xq(xp),cnstr.ub,cnstr.lb,interpOpt{:});

% optimize
[xOpt,objOpt,exitflag,info]=...
    fmincon(fObj,xp0,[],[],[],[],zeros(size(xpUpper)),xpUpper,fCnstr,fminconOpt);
if exitflag>1
    warning(['maxInterpGridData.m: no solution found. exitflag=' num2str(exitflag)])
elseif exitflag<1
    error('no feasible solution found.')
end

% back-scale and expand to full dimensional gridding space
xMax=dimSelVal;
xMax(keepDim_iArr)={nan};
xMax=cell2mat(xMax);
xMax(keepDim_iArr)=unscale(xp2xq(xOpt),xMin,xRange);%   gridMin_arr+xGridPt'.*gridRange_arr;
outMax=unscale(-objOpt,objMin,objRange); % fval is negative!

% plot in case no output arguments are requested
if numel(xDataScaled)==2 && nargout==0
    %% plot results if applicable
    figure;
    xqOpt=xp2xq(xOpt);
    contourf(xDataScaled{1},xDataScaled{2},objDataScaled',100,'LineStyle','none')
    hold on
    for i_cnstr=1:numel(cnstrDataScaled)
        if ~isinf(cnstr.ub(i_cnstr))
            contour(xDataScaled{1},xDataScaled{2},cnstrDataScaled{1}',[cnstr.ub 2],'k')
        end
        if ~isinf(cnstr.lb(i_cnstr))
            contour(xDataScaled{1},xDataScaled{2},cnstrDataScaled{1}',[cnstr.lb 2],'k')
        end
    end

    if exist('xq_cntr','var')
        plot(xq_cntr(1,:),xq_cntr(2,:),'k:');
        % evaluate equality constraint at solution
        disp(['> equality constraint (should be zero): ' ...
            num2str(interpn(x1q(1,:),x2q(:,1),cntrDataScaled-zeroScaled,xqOpt(1),xqOpt(2),interpOpt{:}))]);
    end
    plot(xqOpt(1),xqOpt(2),'rx')
end





%% get maximum grid point of nd data
function [m,xm]=getMaxFeasGrdPntLOCAL(vObj,x,vCnstr,lb,ub)

feasGrdPnt_iArr=true(size(vObj));
for i_cnstr=1:numel(vCnstr)
    vCnstr_act=vCnstr{i_cnstr};
    if (ub(i_cnstr)-lb(i_cnstr))/range(vCnstr_act(:))>0.1
        % only check feasibility if lb and ub are not too narrow 
        % (equality constraints would probably not yield any feasible grid points)
        feasGrdPnt_iArr=feasGrdPnt_iArr & vCnstr_act>=lb(i_cnstr) & vCnstr_act<=ub(i_cnstr);
    end
end
feasGrdPnt_iArr=find(feasGrdPnt_iArr);

if isempty(feasGrdPnt_iArr)
    error('no feasible grid point found. change constraints.')
% elseif numel(feasGrdPnt_iArr)==numel(vObj)
%     disp(['all ' num2str(numel(feasGrdPnt_iArr)) ' grid points feasible..']);
% else
%     disp(['> ' num2str(numel(feasGrdPnt_iArr)) ' grid points feasible..'])
end

[m,i_max]=max(vObj(feasGrdPnt_iArr));
s_max=cell(ndims(vObj)-isvector(vObj),1); % note: special case for vectors!
[s_max{:}]=ind2sub(size(vObj),feasGrdPnt_iArr(i_max));
xm=nan(size(x));
for i_dim=1:numel(x)
    xm(i_dim)=x{i_dim}(s_max{i_dim});
end





%% calculate constraints
function [cVal,ceqVal] = calcConstrLOCAL(x,vCnstr_arr,xq,ub,lb,varargin)


% interpolate outputs at given xq
n_out=numel(vCnstr_arr);
vq=nan(n_out,1);
for i_out=1:n_out
    vq(i_out)=interpnLOCAL(x,vCnstr_arr{i_out},xq,varargin{:});
end

% construct return values
isEqCnstr=(lb==ub);
isLbCnstr=~isinf(lb);
isUbCnstr=~isinf(ub);
ceqVal=vq(isEqCnstr);
cVal=[lb(isLbCnstr)-vq(isLbCnstr);
      vq(isUbCnstr)-ub(isUbCnstr)];







%% interpolation function
function vq=interpnLOCAL(x,v,xq,varargin)

% dimension reduction if necessary (ToDo: do this BEFORE fmincon)
nsDim_iArr=cellfun(@(x) numel(x)>1,x); % non-singular dimensions
v=squeeze(v);
x=x(nsDim_iArr);
xq=num2cell(xq(nsDim_iArr));

% interpolate nd data
try
    vq=interpn(x{:},v,xq{:},varargin{:});
catch
    warning('maxInterpGridData: interpolation failed. replace nan values with min values.')
    v(isnan(v))=min(v(:));
    vq=interpn(x{:},v,xq{:},varargin{:});
end




%% OBSOLETE
% TODO: special treatment of 2D equality constraints using contour
%                 % 1) get GenTq/BldPitchC contour(s) which fullfill control law (i.e. BelowRtdCtrlDiff==0)
%                 [~,~,tmpCtrl,isValid]=getGridData2(data_act,'BelowRtdCtrlDiff','BldPitchC','GenTq',data_act.grid.dimNames(3:end),pVal_arr,false);
%                 tmpCtrl(~isValid)=nan;
%                 ctrlContour=contourc(data_act.grid.BldPitchC,data_act.grid.GenTq,tmpCtrl,[0 inf]); % note: min. 2 countours need to be evaluated (BUG)
%                 % 2) evaluate all contour parts
%                 tmpBldPitchC_arr={};
%                 tmpGenTq_arr={};
%                 while size(ctrlContour,2)>0
%                     if ctrlContour(1,1)==0 % BelowRtdCtrlDiff==0 contour line
%                         tmpBldPitchC_arr{end+1}=ctrlContour(1,2:ctrlContour(2,1)+1); %#ok<AGROW> 
%                         tmpGenTq_arr{end+1}=ctrlContour(2,2:ctrlContour(2,1)+1);     %#ok<AGROW> 
%                     end
%                     ctrlContour(:,1:ctrlContour(2,1)+1)=[];
%                 end
%                 if numel(tmpBldPitchC_arr)~=1
%                     error(['number of control law contours is ' num2str(numel(tmpBldPitchC_arr))])
%                 end
%                 tmpBldPitchC_arr=[tmpBldPitchC_arr{:}];
%                 tmpGenTq_arr=[tmpGenTq_arr{:}];

