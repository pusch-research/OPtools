function [vMax,xMax,yMax]=maxInterp2(v,x,y,fminconOpt,interpOpt)
% interpolated maximum of a 2-dimensional function (v,x,y in ndgrid format)


% handle input
if nargin<2 || isempty(x)
   x=(1:size(v,1))'; % assume unit spacing
end
if size(x,2)==1
   x=repmat(x,1,size(v,2)); % x given as grid vector - transform to ndgrid
end
if nargin<3 || isempty(y)
   y=1:size(v,2); % assume unit spacing
end
if size(y,1)==1
   y=repmat(y,size(v,1),1); % y given as grid vector - transform to ndgrid
end
if nargin<4 || isempty(fminconOpt)
    fminconOpt=optimoptions('fmincon','Display','none','Algorithm','sqp'); % use SQP since interior-point sometimes leads to local optima lower than the initial value
end
if nargin<5 || isempty(interpOpt)
    interpOpt={'spline'};
elseif ~iscell(interpOpt)
    interpOpt={interpOpt};
end

% disable warning
tmpWarnState=warning('query','MATLAB:interpn:NaNstrip');
warning('off','MATLAB:interpn:NaNstrip')

% get limits of x,y,z
x_lim=minmax(x(:)');
y_lim=minmax(y(:)');
v_lim=minmax(v(:)');

% scale
x=(x(:,1)-x_lim(1))/(diff(x_lim)); % get (scaled) vector from 2D grid
y=(y(1,:)'-y_lim(1))/(diff(y_lim)); % get (scaled) vector from 2D grid
v=(v-v_lim(1))/(diff(v_lim));

% objective function
fObj=@(xy) -interpn(x,y,v,xy(1),xy(2),interpOpt{:});

% initial value
[~,i_xyMax]=max(v(:));
[i_xMax,i_yMax]=ind2sub(size(v),i_xyMax);
x0=[x(i_xMax) y(i_yMax)];

% optimize
[xyMax,fval,exitflag]=...
    fmincon(fObj,x0,[],[],[],[],[min(x) min(y)],[max(x) max(y)],[],fminconOpt);
if exitflag~=1
    warning(['maxInterp2.m: no solution found. exitflag=' num2str(exitflag)])
end

% back-scale
xMax=x_lim(1)+xyMax(1)*diff(x_lim);
yMax=y_lim(1)+xyMax(2)*diff(y_lim);
vMax=v_lim(1)-fval*diff(v_lim); % fval is negative

% reset warning state
warning(tmpWarnState.state,'MATLAB:interpn:NaNstrip');