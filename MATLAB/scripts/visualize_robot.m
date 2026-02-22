function visualize_robot(varargin)
% VISUALIZE_ROBOT  Interactive 3-D viewer for the ME557 arm — NO TOOLBOX.
%
% Loads the URDF, builds the PoE model, and opens a figure with:
%   - Stick-figure skeleton (joint origins connected by lines)
%   - Optional STL mesh overlay (auto-detected if meshes/ folder exists)
%   - Joint-angle sliders with live FK update
%   - Screw-axis arrows at every joint
%   - Console diagnostics: M, Slist, joint limits, Jacobian, etc.
%
% USAGE (from the MATLAB/scripts/ directory):
%   visualize_robot                          % defaults
%   visualize_robot('meshes', false)         % skip STL loading
%   visualize_robot('target', [0.2; 0; 0.3])% IK test to a point
%   visualize_robot('urdf', 'path/to.urdf') % override URDF
%
% REQUIRES:
%   robot_model_from_urdf.m   (already in your repo)
%   Modern Robotics /mr folder (FKinSpace, IKinSpace, JacobianSpace …)
%   robot_joint_limits.m      (from the gap-analysis delivery)
%
% OPTIONAL:
%   STL mesh files in  simple_robot_v2/meshes/  (auto-detected)
%
% Tested on R2020b+.  stlread() is built-in since R2018b.

% =====================================================================
%  0.  Parse optional arguments
% =====================================================================
p = inputParser;
addParameter(p, 'meshes',  true);
addParameter(p, 'target',  []);
addParameter(p, 'urdf',    '');
addParameter(p, 'meshdir', '');
parse(p, varargin{:});
opts = p.Results;

% =====================================================================
%  1.  Paths — mirror main_writeboard_demo.m logic
% =====================================================================
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

addpath(genpath(fullfile(matlabDir, 'mr')));
addpath(scriptsDir);

if isempty(opts.urdf)
    urdfPath = fullfile(matlabDir, 'simple_robot_v2.urdf');
else
    urdfPath = opts.urdf;
end

% =====================================================================
%  2.  Build PoE model (uses YOUR existing function)
% =====================================================================
baseLink   = "FixedBase";
tipLink    = "PenTipLink";
jointNames = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint", ...
              "WristJoint","PenJoint"];

fprintf('\n========== LOADING URDF ==========\n  %s\n', urdfPath);
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames);

nJ = numel(jointNames);
fprintf('\nHome transform M (base -> tip at theta = 0):\n');
disp(robot.M);
fprintf('Screw axes Slist (6 x %d):\n', nJ);
for j = 1:nJ
    fprintf('  S%d = [%8.4f %8.4f %8.4f | %8.4f %8.4f %8.4f]  (%s)\n', ...
        j, robot.Slist(:,j)', jointNames(j));
end

% =====================================================================
%  3.  Joint limits
% =====================================================================
try
    limits = robot_joint_limits();
    hasLimits = true;
    fprintf('\nJoint limits (from robot_joint_limits.m):\n');
catch
    warning('robot_joint_limits.m not on path — using +/-pi.');
    limits.lower = -pi*ones(nJ,1);
    limits.upper =  pi*ones(nJ,1);
    limits.names = jointNames(:);
    hasLimits = false;
end
for j = 1:nJ
    fprintf('  J%d %-16s  [%+7.2f, %+7.2f] deg\n', j, limits.names(j), ...
        rad2deg(limits.lower(j)), rad2deg(limits.upper(j)));
end

% =====================================================================
%  4.  Parse URDF chain for drawing (need intermediate joint origins)
% =====================================================================
chain = parse_chain(urdfPath, baseLink, tipLink);
fprintf('\nURDF chain: %d joints from %s -> %s\n', numel(chain), baseLink, tipLink);

% Map actuated joint names -> chain indices
actIdx = zeros(nJ,1);
for j = 1:nJ
    for c = 1:numel(chain)
        if chain(c).name == jointNames(j)
            actIdx(j) = c;
            break;
        end
    end
end

% =====================================================================
%  5.  Load STL meshes (optional)
% =====================================================================
meshes = struct('link',{},'verts',{},'faces',{},'color',{});
if opts.meshes
    meshDir = find_mesh_dir(opts.meshdir, matlabDir);
    if ~isempty(meshDir)
        meshes = load_meshes(urdfPath, meshDir);
        fprintf('Loaded %d STL meshes from %s\n', numel(meshes), meshDir);
    else
        fprintf('Mesh folder not found — stick-figure only.\n');
    end
end

% =====================================================================
%  6.  Diagnostics at home
% =====================================================================
theta = robot.home;
J0 = JacobianSpace(robot.Slist, theta);
fprintf('\nSpace Jacobian at home (6 x %d):\n', nJ);
disp(round(J0, 6));
fprintf('Manipulability index: %.6f\n', sqrt(max(0,det(J0*J0'))));

T_home = FKinSpace(robot.M, robot.Slist, theta);
fprintf('Home tip position: [%.5f, %.5f, %.5f] m\n', T_home(1:3,4));

% Approx reach
reach = sum(arrayfun(@(c) norm(c.xyz), chain));
fprintf('Estimated reach: %.4f m\n', reach);

% =====================================================================
%  7.  IK test (if target supplied)
% =====================================================================
ikSolution = [];
if ~isempty(opts.target)
    pt = opts.target(:);
    Tgoal = T_home;              % keep home orientation
    Tgoal(1:3,4) = pt;
    fprintf('\n--- IK TEST ---\nTarget: [%.4f, %.4f, %.4f]\n', pt);
    [thetaIK, ok] = IKinSpace(robot.Slist, robot.M, Tgoal, ...
                               theta, 1e-3, 1e-3);
    if ok
        fprintf('  SOLVED (deg): [%s]\n', num2str(rad2deg(thetaIK'),'%.1f  '));
        if hasLimits
            oob = thetaIK < limits.lower | thetaIK > limits.upper;
            if any(oob)
                fprintf('  WARNING: joints [%s] outside limits!\n', ...
                    num2str(find(oob)'));
            end
        end
        theta = thetaIK;
        ikSolution = thetaIK;
    else
        fprintf('  FAILED — target may be unreachable.\n');
    end
end

% =====================================================================
%  8.  Build interactive figure
% =====================================================================
fig = figure('Name','ME557 Robot Viewer','NumberTitle','off', ...
             'Position',[80 80 1050 780], 'Color','w');

ax3d = axes('Parent',fig,'Position',[0.05 0.30 0.90 0.67]);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
xlabel(ax3d,'X (m)'); ylabel(ax3d,'Y (m)'); zlabel(ax3d,'Z (m)');
title(ax3d,'ME557 Arm — Interactive FK / IK Viewer (No Toolbox)');
view(ax3d, 135, 25);

axLim = reach * 1.4;
set(ax3d,'XLim',[-axLim axLim],'YLim',[-axLim axLim],'ZLim',[-0.02 axLim]);

% Ground plane
fill3(ax3d, axLim*[-1 1 1 -1], axLim*[-1 -1 1 1], [0 0 0 0], ...
      [0.92 0.92 0.92], 'FaceAlpha',0.25,'EdgeColor','none');

% Graphics handles — skeleton
hLine = plot3(ax3d, nan,nan,nan,'k-o','LineWidth',2.5,'MarkerSize',8, ...
              'MarkerFaceColor',[0.2 0.5 1]);
hTip  = plot3(ax3d, nan,nan,nan,'rp','MarkerSize',16,'MarkerFaceColor','r');

% IK target marker
if ~isempty(opts.target)
    pt = opts.target(:);
    plot3(ax3d, pt(1),pt(2),pt(3),'gx','MarkerSize',20,'LineWidth',3);
end

hArrows = gobjects(0);
hPatch  = gobjects(0);
colors  = lines(nJ);

% Info bar
hInfo = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.01 0.23 0.98 0.05],'FontName','Consolas','FontSize',9, ...
    'HorizontalAlignment','left','BackgroundColor','w');

% Sliders
sliders  = gobjects(nJ,1);
slLabels = gobjects(nJ,1);
y0 = 0.17;  h = 0.028;  gap = 0.002;

for j = 1:nJ
    yy = y0 - (j-1)*(h+gap);
    lo = limits.lower(j); hi = limits.upper(j);
    if lo >= hi, lo=-pi; hi=pi; end
    val = max(lo, min(hi, theta(j)));

    slLabels(j) = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.01 yy 0.17 h], ...
        'String',sprintf('J%d %s: %+.1f°',j,limits.names(j),rad2deg(val)), ...
        'FontSize',8,'HorizontalAlignment','right', ...
        'ForegroundColor',colors(j,:),'BackgroundColor','w');

    sliders(j) = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.19 yy 0.60 h],'Min',lo,'Max',hi,'Value',val);

    addlistener(sliders(j),'Value','PostSet',@(~,~)onSlider());

    % Numeric readout on the right
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.80 yy 0.09 h], ...
        'String',sprintf('[%.0f, %.0f]',rad2deg(lo),rad2deg(hi)), ...
        'FontSize',7,'HorizontalAlignment','center','BackgroundColor','w');
end

% Add a "Home" button
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.90 0.17 0.09 0.04],'String','HOME', ...
    'Callback',@(~,~)goHome());

camlight(ax3d,'headlight');

% Initial draw
redraw();
fprintf('\n========== VIEWER READY ==========\n');
fprintf('Drag sliders to move joints.  Press HOME to reset.\n\n');

% =================================================================
%  NESTED CALLBACKS
% =================================================================
    function onSlider()
        for jj=1:nJ
            theta(jj) = get(sliders(jj),'Value');
            set(slLabels(jj),'String', ...
                sprintf('J%d %s: %+.1f°',jj,limits.names(jj),rad2deg(theta(jj))));
        end
        redraw();
    end

    function goHome()
        theta = robot.home;
        for jj=1:nJ
            set(sliders(jj),'Value',0);
            set(slLabels(jj),'String', ...
                sprintf('J%d %s:  0.0°',jj,limits.names(jj)));
        end
        redraw();
    end

    function redraw()
        % --- FK through chain ---
        [jPos, ~] = chain_fk(chain, jointNames, theta);
        T_tip_now = FKinSpace(robot.M, robot.Slist, theta);
        tip = T_tip_now(1:3,4);

        pts = [[0;0;0], jPos, tip];
        set(hLine,'XData',pts(1,:),'YData',pts(2,:),'ZData',pts(3,:));
        set(hTip, 'XData',tip(1),'YData',tip(2),'ZData',tip(3));

        % --- Screw-axis arrows ---
        delete(hArrows); hArrows = gobjects(nJ,1);
        Jnow = JacobianSpace(robot.Slist, theta);
        aLen = reach*0.10;
        for jj = 1:nJ
            if jj <= size(jPos,2)
                o = jPos(:,jj);
                w = Jnow(1:3,jj);
                w = w/(norm(w)+1e-12);
                hArrows(jj) = quiver3(ax3d, o(1),o(2),o(3), ...
                    w(1)*aLen, w(2)*aLen, w(3)*aLen, 0, ...
                    'Color',colors(jj,:),'LineWidth',2,'MaxHeadSize',0.5);
            end
        end

        % --- STL meshes ---
        delete(hPatch); hPatch = gobjects(0);
        if ~isempty(meshes)
            linkFrames = all_link_frames(chain, jointNames, theta);
            for mm = 1:numel(meshes)
                lname = meshes(mm).link;
                if isKey(linkFrames, lname)
                    Tl = linkFrames(lname);
                else
                    Tl = eye(4);
                end
                vW = (Tl(1:3,1:3) * meshes(mm).verts' + Tl(1:3,4))';
                hp = patch(ax3d,'Faces',meshes(mm).faces,'Vertices',vW, ...
                    'FaceColor',meshes(mm).color,'EdgeColor','none', ...
                    'FaceAlpha',0.4,'FaceLighting','gouraud');
                hPatch(end+1) = hp; %#ok<AGROW>
            end
        end

        % --- Info bar ---
        manip = sqrt(max(0,det(Jnow*Jnow')));
        set(hInfo,'String',sprintf( ...
            'Tip: [%+.4f, %+.4f, %+.4f] m  |  Manip: %.5f  |  θ(°): [%s]', ...
            tip, manip, num2str(round(rad2deg(theta'),1),'%+.1f  ')));
    end

end  % === END main function ===


% #####################################################################
%  LOCAL HELPER FUNCTIONS
% #####################################################################

function chain = parse_chain(urdfPath, baseLink, tipLink)
% Returns struct array:  chain(i).name, .type, .parent, .child, .xyz, .rpy, .axis

doc = xmlread(urdfPath);
jNodes = doc.getElementsByTagName("joint");

jmap = containers.Map;
for i = 0:jNodes.getLength-1
    nd = jNodes.item(i);
    nm = string(nd.getAttribute("name"));
    s.name   = nm;
    s.type   = string(nd.getAttribute("type"));
    s.parent = string(nd.getElementsByTagName("parent").item(0).getAttribute("link"));
    s.child  = string(nd.getElementsByTagName("child").item(0).getAttribute("link"));
    oN = nd.getElementsByTagName("origin");
    if oN.getLength>0
        s.xyz = sscanf(char(oN.item(0).getAttribute("xyz")),'%f')';
        s.rpy = sscanf(char(oN.item(0).getAttribute("rpy")),'%f')';
    else
        s.xyz = [0 0 0]; s.rpy = [0 0 0];
    end
    if numel(s.xyz)<3, s.xyz(end+1:3)=0; end
    if numel(s.rpy)<3, s.rpy(end+1:3)=0; end
    aN = nd.getElementsByTagName("axis");
    if aN.getLength>0
        s.axis = sscanf(char(aN.item(0).getAttribute("xyz")),'%f')';
    else
        s.axis = [0 0 0];
    end
    if numel(s.axis)<3, s.axis(end+1:3)=0; end
    jmap(nm) = s;
end

c2j = containers.Map;
ks = jmap.keys;
for k=1:numel(ks), jj=jmap(ks{k}); c2j(jj.child)=jj.name; end

names = strings(0);
lk = string(tipLink);
while lk ~= string(baseLink)
    names(end+1) = c2j(lk); %#ok<AGROW>
    lk = jmap(c2j(lk)).parent;
end
names = flip(names);

chain = repmat(struct('name',"","type","","parent","","child","", ...
    'xyz',[0 0 0],'rpy',[0 0 0],'axis',[0 0 0]), numel(names), 1);
for i=1:numel(names)
    chain(i) = jmap(names(i));
end
end


function [positions, frames] = chain_fk(chain, actNames, theta)
% Forward kinematics through the URDF chain.
% Returns positions (3 x nActuated) and frames (cell array of 4x4).

nA = numel(actNames);
positions = zeros(3, nA);
frames    = cell(nA, 1);
T = eye(4);

for i = 1:numel(chain)
    T = T * rpy_xyz_to_T(chain(i).rpy, chain(i).xyz);

    % Check if this is one of the actuated joints
    for a = 1:nA
        if chain(i).name == actNames(a)
            positions(:,a) = T(1:3,4);
            frames{a} = T;
            % Apply the joint rotation
            T = T * axis_rot_T(chain(i).axis, theta(a));
            break;
        end
    end
end
end


function linkFrames = all_link_frames(chain, actNames, theta)
% Returns containers.Map:  childLinkName -> 4x4 world-frame transform.

nA = numel(actNames);
linkFrames = containers.Map;
linkFrames(chain(1).parent) = eye(4);   % base link
T = eye(4);

for i = 1:numel(chain)
    T = T * rpy_xyz_to_T(chain(i).rpy, chain(i).xyz);
    for a = 1:nA
        if chain(i).name == actNames(a)
            T = T * axis_rot_T(chain(i).axis, theta(a));
            break;
        end
    end
    linkFrames(chain(i).child) = T;
end
end


function ms = load_meshes(urdfPath, meshDir)
% Load STL files referenced in URDF, return struct array.

doc = xmlread(urdfPath);
linkNodes = doc.getElementsByTagName("link");

% Nice colors per link
cmap = containers.Map;
cmap("FixedBase")        = [0.60 0.60 0.60];
cmap("RotatingBaseLink") = [0.50 0.50 1.00];
cmap("ShoulderLink")     = [0.79 0.82 0.93];
cmap("ElbowLink")        = [0.79 0.82 0.93];
cmap("WristLink")        = [0.91 0.44 0.03];
cmap("PenLink")          = [0.50 1.00 0.50];
cmap("PenTipLink")       = [0.50 1.00 0.50];

ms = struct('link',{},'verts',{},'faces',{},'color',{});

for i = 0:linkNodes.getLength-1
    lk = linkNodes.item(i);
    lname = string(lk.getAttribute("name"));

    vis = lk.getElementsByTagName("visual");
    if vis.getLength==0, continue; end
    msh = vis.item(0).getElementsByTagName("mesh");
    if msh.getLength==0, continue; end

    fname = string(msh.item(0).getAttribute("filename"));
    parts = split(fname,'/');
    stlFile = fullfile(meshDir, parts(end));

    if ~isfile(stlFile), continue; end

    try
        TR = stlread(stlFile);
        if isKey(cmap,lname), col=cmap(lname); else, col=[0.7 0.7 0.7]; end
        ms(end+1) = struct('link',lname,'verts',TR.Points, ...
                           'faces',TR.ConnectivityList,'color',col); %#ok<AGROW>
    catch
    end
end
end


function meshDir = find_mesh_dir(override, matlabDir)
if ~isempty(override) && isfolder(override)
    meshDir = override; return;
end
candidates = { ...
    fullfile(matlabDir, 'simple_robot_v2', 'meshes'), ...
    fullfile(matlabDir, 'meshes'), ...
    fullfile(matlabDir, '..', 'simple_robot_v2', 'meshes'), ...
    fullfile(matlabDir, '..', 'simple_robot_v2', 'urdf', 'meshes')};
meshDir = '';
for c = 1:numel(candidates)
    if isfolder(candidates{c})
        meshDir = candidates{c};
        return;
    end
end
end


function T = rpy_xyz_to_T(rpy, xyz)
r=rpy(1); p=rpy(2); y=rpy(3);
Rx=[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
Ry=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
Rz=[cos(y) -sin(y) 0;sin(y) cos(y) 0;0 0 1];
T=[Rz*Ry*Rx, xyz(:); 0 0 0 1];
end


function T = axis_rot_T(ax, angle)
% Rodrigues rotation as a 4x4 homogeneous matrix.
ax = ax(:);
n = norm(ax);
if n < 1e-12, T = eye(4); return; end
ax = ax/n;
K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
T = [R [0;0;0]; 0 0 0 1];
end