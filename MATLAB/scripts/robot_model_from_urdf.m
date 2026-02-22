function robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNamesOrdered)
% Builds a PoE model (M, Slist) from a URDF chain (NO Robotics Toolbox).
% Uses Modern Robotics convention: S = [w; v], v = -w x q.

doc = xmlread(urdfPath);

% --- Parse all joints ---
jointNodes = doc.getElementsByTagName("joint");
jmap = containers.Map;

for i = 0:jointNodes.getLength-1
    j = jointNodes.item(i);
    name = string(j.getAttribute("name"));
    type = string(j.getAttribute("type"));

    parent = string(j.getElementsByTagName("parent").item(0).getAttribute("link"));
    child  = string(j.getElementsByTagName("child").item(0).getAttribute("link"));

    % origin
    originNode = j.getElementsByTagName("origin");
    if originNode.getLength > 0
        o = originNode.item(0);
        xyz = parseVec3(string(o.getAttribute("xyz")), [0 0 0]);
        rpy = parseVec3(string(o.getAttribute("rpy")), [0 0 0]);
    else
        xyz = [0 0 0]; rpy = [0 0 0];
    end

    % axis
    axisNode = j.getElementsByTagName("axis");
    if axisNode.getLength > 0
        a = axisNode.item(0);
        axis = parseVec3(string(a.getAttribute("xyz")), [0 0 1]);
    else
        axis = [0 0 0];
    end

    jmap(name) = struct( ...
        "name",name,"type",type,"parent",parent,"child",child, ...
        "xyz",xyz(:),"rpy",rpy(:),"axis",axis(:));
end

% --- Build child->joint lookup ---
childToJoint = containers.Map;
keysJ = jmap.keys;
for k = 1:numel(keysJ)
    jj = jmap(keysJ{k});
    childToJoint(jj.child) = jj.name;
end

% --- Walk from tip back to base to find chain ---
chainJoints = strings(0);
link = string(tipLink);
while link ~= string(baseLink)
    if ~isKey(childToJoint, link)
        error("Cannot find joint whose child link is '%s'. Check base/tip names.", link);
    end
    jname = childToJoint(link);
    chainJoints(end+1) = jname; %#ok<AGROW>
    link = jmap(jname).parent;
end
chainJoints = flip(chainJoints);

% --- Compute home transforms and screw axes ---
T = eye(4);
T_to_joint = containers.Map;

for idx = 1:numel(chainJoints)
    j = jmap(chainJoints(idx));
    T_joint = T * T_origin_rpy_xyz(j.rpy, j.xyz);
    T_to_joint(j.name) = T_joint;

    % advance along chain at home (theta=0)
    T = T_joint;
end

M = T; % base->tip at home

Slist = zeros(6, numel(jointNamesOrdered));
for i = 1:numel(jointNamesOrdered)
    jn = jointNamesOrdered(i);
    if ~isKey(jmap, jn)
        error("Joint '%s' not found in URDF. Update jointNamesOrdered.", jn);
    end
    j = jmap(jn);

    if j.type ~= "revolute"
        error("Joint '%s' is not revolute (type=%s).", jn, j.type);
    end

    Tj = T_to_joint(j.name);
    R  = Tj(1:3,1:3);
    p  = Tj(1:3,4);

    w = R * Normalize(j.axis);
    q = p;
    v = -cross(w, q);

    Slist(:,i) = [w; v];
end

robot = struct();
robot.urdfPath   = urdfPath;
robot.baseLink   = baseLink;
robot.tipLink    = tipLink;
robot.jointNames = jointNamesOrdered(:);
robot.Slist      = Slist;
robot.M          = M;

% Your MoveIt SRDF "home" is zeros; for this repo we keep that convention.
robot.home = zeros(numel(jointNamesOrdered), 1);
end

% ---- helpers ----
function v = parseVec3(s, default)
s = strtrim(s);
if s == ""
    v = default;
    return;
end
parts = double(string(split(s)));
v = [parts(1) parts(2) parts(3)];
end

function T = T_origin_rpy_xyz(rpy, xyz)
R = RzRyRx(rpy(3), rpy(2), rpy(1));
T = [R, xyz(:); 0 0 0 1];
end

function R = RzRyRx(yaw, pitch, roll)
% URDF: R = Rz(yaw)*Ry(pitch)*Rx(roll)
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
R = Rz*Ry*Rx;
end