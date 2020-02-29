function showPath(wps)
% Displays PRM planner with calculated path
% Copyright 2019 The MathWorks, Inc.

figure(gcf);

persistent h
if isempty(h)
    hold on
    h = plot(wps(:,1),wps(:,2),'r:','LineWidth',2);
    hold off
else
    set(h,'XData',wps(:,1),'YData',wps(:,2));
end

end

