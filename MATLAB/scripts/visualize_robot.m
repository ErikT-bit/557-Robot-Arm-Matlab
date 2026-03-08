function viewer = visualize_robot(varargin)
% VISUALIZE_ROBOT
% Live 3-D viewer for the ME557 arm, designed to run alongside
% main_writeboard_demo.
%
% Returns a struct of function handles:
%   viewer.update(theta)
%   viewer.setBoard(center, xhat, yhat, width_m, height_m)
%   viewer.setPath(pathPts)
%   tf = viewer.waitForEnterOrQ(promptStr)
%   viewer.clearPath()
%   viewer.close()
%
% If called with no output, it still opens the viewer.

% =====================================================================
% 0) Parse args
% =====================================================================
p = inputParser;
addParameter(p,'robot',[]);
addParameter(p,'urdf','');
addParameter(p,'theta0',zeros(5,1));
addParameter(p,'title','ME557 Robot Viewer');
parse(p,varargin{:});
opts = p.Results;

% =====================================================================
% 1) Paths / model
% =====================================================================
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

addpath(genpath(fullfile(matlabDir,'mr')));
addpath(scriptsDir);

if isempty(opts.robot)
    if isempty(opts.urdf)
        urdfPath = fullfile(matlabDir,'simple_robot_v2.urdf');
    else
        urdfPath = opts.urdf;
    end

    baseLink   = "FixedBase";
    tipLink    = "PenTipLink";
    jointNames = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];
    robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames);
else
    robot = opts.robot;
    jointNames = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];
    if isempty(opts.urdf)
        urdfPath = fullfile(matlabDir,'simple_robot_v2.urdf');
    else
        urdfPath = opts.urdf;
    end
end

chain = parse_chain(urdfPath, "FixedBase", "PenTipLink");

% Rough reach for axes limits
reach = 0;
for i = 1:numel(chain)
    reach = reach + norm(chain(i).xyz);
end
reach = max(reach, 0.4);

theta = opts.theta0(:);

% =====================================================================
% 2) Figure / graphics objects
% =====================================================================
fig = figure( ...
    'Name', opts.title, ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [80 80 1100 760], ...
    'KeyPressFcn', @onKeyPress, ...
    'CloseRequestFcn', @onClose);

ax = axes('Parent',fig,'Position',[0.05 0.08 0.90 0.86]);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
xlabel(ax,'X (m)');
ylabel(ax,'Y (m)');
zlabel(ax,'Z (m)');
title(ax, opts.title);
view(ax,135,25);

axLim = 1.4 * reach;
set(ax,'XLim',[-axLim axLim], ...
       'YLim',[-axLim axLim], ...
       'ZLim',[-0.05 axLim]);

% Ground
fill3(ax, ...
    axLim*[-1 1 1 -1], ...
    axLim*[-1 -1 1 1], ...
    [0 0 0 0], ...
    [0.93 0.93 0.93], ...
    'FaceAlpha',0.35,'EdgeColor','none');

% Robot skeleton
hLine = plot3(ax,nan,nan,nan,'k-o', ...
    'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.2 0.5 1]);
hTip  = plot3(ax,nan,nan,nan,'rp', ...
    'MarkerSize',14,'MarkerFaceColor','r');

% Board rectangle
hBoard = patch(ax, ...
    'XData',nan,'YData',nan,'ZData',nan, ...
    'FaceColor',[0.2 0.8 0.2], ...
    'FaceAlpha',0.15, ...
    'EdgeColor',[0.0 0.55 0.0], ...
    'LineWidth',2);

% Board center marker
hBoardCtr = plot3(ax,nan,nan,nan,'go','MarkerSize',8,'LineWidth',2);

% Planned writing path
hPath = plot3(ax,nan,nan,nan,'m-','LineWidth',2);

% Current tip trace (optional short history)
hTrace = plot3(ax,nan,nan,nan,'c-','LineWidth',1);
tipTrace = zeros(3,0);
maxTrace = 300;

% Info text
hInfo = uicontrol(fig, ...
    'Style','text', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.98 0.05], ...
    'FontName','Consolas', ...
    'FontSize',10, ...
    'HorizontalAlignment','left', ...
    'BackgroundColor','w', ...
    'String','Viewer ready.');

camlight(ax,'headlight');

setappdata(fig,'lastKey','');
setappdata(fig,'isClosed',false);

% Initial draw
redrawRobot(theta);

% =====================================================================
% 3) Return viewer API
% =====================================================================
viewer.fig          = fig;
viewer.ax           = ax;
viewer.update       = @updateTheta;
viewer.setBoard     = @setBoard;
viewer.setPath      = @setPath;
viewer.clearPath    = @clearPath;
viewer.waitForEnterOrQ = @waitForEnterOrQ;
viewer.close        = @closeViewer;

if nargout == 0
    clear viewer
end

% =====================================================================
% Nested functions
% =====================================================================
    function updateTheta(thetaNow)
        if ~isgraphics(fig), return; end
        theta = thetaNow(:);
        redrawRobot(theta);
        drawnow limitrate nocallbacks;
    end

    function setBoard(center, xhat, yhat, width_m, height_m)
        if ~isgraphics(fig), return; end
        center = center(:);
        xhat = xhat(:) / norm(xhat);
        yhat = yhat(:) / norm(yhat);

        hx = 0.5 * width_m;
        hy = 0.5 * height_m;

        c1 = center - hx*xhat - hy*yhat;
        c2 = center + hx*xhat - hy*yhat;
        c3 = center + hx*xhat + hy*yhat;
        c4 = center - hx*xhat + hy*yhat;

        set(hBoard, ...
            'XData',[c1(1) c2(1) c3(1) c4(1)], ...
            'YData',[c1(2) c2(2) c3(2) c4(2)], ...
            'ZData',[c1(3) c2(3) c3(3) c4(3)]);
        set(hBoardCtr, ...
            'XData',center(1), ...
            'YData',center(2), ...
            'ZData',center(3));
        drawnow limitrate nocallbacks;
    end

    function setPath(pathPts)
        if ~isgraphics(fig), return; end
        if isempty(pathPts)
            set(hPath,'XData',nan,'YData',nan,'ZData',nan);
        else
            set(hPath, ...
                'XData',pathPts(1,:), ...
                'YData',pathPts(2,:), ...
                'ZData',pathPts(3,:));
        end
        drawnow limitrate nocallbacks;
    end

    function clearPath()
        if ~isgraphics(fig), return; end
        set(hPath,'XData',nan,'YData',nan,'ZData',nan);
        drawnow limitrate nocallbacks;
    end

    function tf = waitForEnterOrQ(promptStr)
        if nargin < 1
            promptStr = 'Press ENTER to continue, or q to abort.';
        end
        if ~isgraphics(fig)
            tf = false;
            return;
        end

        setappdata(fig,'lastKey','');
        set(hInfo,'String',promptStr);

        tf = false;
        while isgraphics(fig)
            key = getappdata(fig,'lastKey');
            if strcmp(key,'return') || strcmp(key,'enter')
                tf = true;
                break
            elseif strcmpi(key,'q')
                tf = false;
                break
            end
            pause(0.03);
        end

        if isgraphics(fig)
            setappdata(fig,'lastKey','');
        end
    end

    function redrawRobot(thetaNow)
        jPos = chain_fk(chain, jointNames, thetaNow);
        Ttip = FKinSpace(robot.M, robot.Slist, thetaNow);
        tip = Ttip(1:3,4);

        pts = [[0;0;0], jPos, tip];
        set(hLine,'XData',pts(1,:),'YData',pts(2,:),'ZData',pts(3,:));
        set(hTip ,'XData',tip(1),'YData',tip(2),'ZData',tip(3));

        tipTrace(:,end+1) = tip;
        if size(tipTrace,2) > maxTrace
            tipTrace = tipTrace(:,end-maxTrace+1:end);
        end
        set(hTrace,'XData',tipTrace(1,:),'YData',tipTrace(2,:),'ZData',tipTrace(3,:));

        set(hInfo,'String',sprintf( ...
            'Tip: [%+.4f, %+.4f, %+.4f] m   |   theta(deg): [%s]   |   ENTER=continue, q=abort', ...
            tip(1), tip(2), tip(3), num2str(round(rad2deg(thetaNow(:)'),1),'%+.1f  ')));
    end

    function onKeyPress(~, evt)
        if isempty(evt) || ~isfield(evt,'Key'), return; end
        setappdata(fig,'lastKey',evt.Key);
    end

    function closeViewer()
        if isgraphics(fig)
            delete(fig);
        end
    end

    function onClose(~,~)
        setappdata(fig,'isClosed',true);
        delete(fig);
    end
end

% #####################################################################
% LOCAL HELPERS
% #####################################################################

function chain = parse_chain(urdfPath, baseLink, tipLink)
doc = xmlread(urdfPath);
jNodes = doc.getElementsByTagName("joint");

jmap = containers.Map;
for i = 0:jNodes.getLength-1
    nd = jNodes.item(i);
    nm = char(nd.getAttribute("name"));
    s = struct();
    s.name   = nm;
    s.type   = char(nd.getAttribute("type"));
    s.parent = char(nd.getElementsByTagName("parent").item(0).getAttribute("link"));
    s.child  = char(nd.getElementsByTagName("child").item(0).getAttribute("link"));

    oN = nd.getElementsByTagName("origin");
    if oN.getLength > 0
        s.xyz = safe_vec3(char(oN.item(0).getAttribute("xyz")));
        s.rpy = safe_vec3(char(oN.item(0).getAttribute("rpy")));
    else
        s.xyz = [0 0 0];
        s.rpy = [0 0 0];
    end

    aN = nd.getElementsByTagName("axis");
    if aN.getLength > 0
        s.axis = safe_vec3(char(aN.item(0).getAttribute("xyz")));
    else
        s.axis = [0 0 0];
    end

    jmap(nm) = s;
end

c2j = containers.Map;
ks = jmap.keys;
for k = 1:numel(ks)
    jj = jmap(ks{k});
    c2j(jj.child) = jj.name;
end

names = {};
lk = char(tipLink);
while ~strcmp(lk, char(baseLink))
    jname = c2j(lk);
    names{end+1} = jname; %#ok<AGROW>
    lk = jmap(jname).parent;
end
names = flip(names);

chain = struct('name',{},'type',{},'parent',{},'child',{}, ...
               'xyz',{},'rpy',{},'axis',{});
for i = 1:numel(names)
    chain(i) = jmap(names{i});
end
end

function positions = chain_fk(chain, actNames, theta)
nA = numel(actNames);
positions = zeros(3,nA);
T = eye(4);

for i = 1:numel(chain)
    T = T * rpy_xyz_to_T(chain(i).rpy, chain(i).xyz);

    for a = 1:nA
        if strcmp(chain(i).name, char(actNames(a)))
            positions(:,a) = T(1:3,4);
            T = T * axis_rot_T(chain(i).axis, theta(a));
            break;
        end
    end
end
end

function T = rpy_xyz_to_T(rpy, xyz)
r = rpy(1); p = rpy(2); y = rpy(3);
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
T = [Rz*Ry*Rx, xyz(:); 0 0 0 1];
end

function T = axis_rot_T(ax, angle)
ax = ax(:);
n = norm(ax);
if n < 1e-12
    T = eye(4);
    return
end
ax = ax / n;
K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
T = [R [0;0;0]; 0 0 0 1];
end

function v = safe_vec3(s)
s = strtrim(s);
if isempty(s)
    v = [0 0 0];
    return
end
v = sscanf(s,'%f')';
if numel(v) < 3
    v(end+1:3) = 0;
end
end