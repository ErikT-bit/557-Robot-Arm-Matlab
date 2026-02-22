function strokes = sketchpad_capture()
% Mouse sketch capture:
% - Hold LEFT mouse button = pen down (draw)
% - Release = pen up (stroke ends)
% - Press ENTER to finish
%
% Output: strokes is a cell array where each element has:
%   strokes{k}.xy      -> 2xN points in figure coordinates
%   strokes{k}.penDown -> true (segment represents contact stroke)

fig = figure("Name","Sketchpad (hold LMB to draw, release = pen up, Enter = done)");
axis equal; grid on; hold on;
xlabel("x"); ylabel("y");
title("Hold LEFT mouse to draw. Release = pen-up. Press ENTER to finish.");

strokes = {};
current = [];
isDown = false;

set(fig, "WindowButtonDownFcn", @onDown);
set(fig, "WindowButtonUpFcn",   @onUp);
set(fig, "WindowButtonMotionFcn", @onMove);
set(fig, "KeyPressFcn", @onKey);

uiwait(fig);

    function onDown(~,~)
        isDown = true;
        cp = get(gca,"CurrentPoint"); 
        p = cp(1,1:2).';
        current = p;
        plot(p(1), p(2), ".", "MarkerSize", 10);
    end

    function onMove(~,~)
        if ~isDown, return; end
        cp = get(gca,"CurrentPoint"); 
        p = cp(1,1:2).';
        current(:,end+1) = p; %#ok<AGROW>
        plot(current(1,end-1:end), current(2,end-1:end), "-");
        drawnow limitrate;
    end

    function onUp(~,~)
        if ~isDown, return; end
        isDown = false;

        if size(current,2) >= 2
            s = struct();
            s.xy = current;
            s.penDown = true;
            strokes{end+1} = s; %#ok<AGROW>
        end

        current = [];
    end

    function onKey(~,evt)
        if strcmp(evt.Key,"return")
            uiresume(fig);
            close(fig);
        end
    end
end