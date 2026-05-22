clear;
clc;

waypoints = load('D:\620__IVS\2nd__period\CM__Projects\Practice_solution\src_cm4sl\map_data\final\Final_ver_waypoints.mat');
segments = load('D:\620__IVS\2nd__period\CM__Projects\Practice_solution\src_cm4sl\map_data\final\Final_ver_roadprofiles.mat');

hold on;    
for i = 1:size(waypoints.ids,1)
    plot(waypoints.waypoints(i,1),waypoints.waypoints(i,2),'k.', 'MarkerSize', 10);
    text(waypoints.waypoints(i,1)-0.5, waypoints.waypoints(i,2)-1, num2str(waypoints.ids(i)), 'FontSize', 6, 'Color', 'k');
end

for i = 1:size(segments.ids,1)
    temp = segments.waypoints(i,:);
    temp = temp(temp~=0);

    for j = 1:length(temp)-1
        plot([waypoints.waypoints(temp(j),1),waypoints.waypoints(temp(j+1),1)], [waypoints.waypoints(temp(j),2),waypoints.waypoints(temp(j+1),2)],'c-');
    end

    plot(waypoints.waypoints(temp(1), 1), waypoints.waypoints(temp(1), 2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(waypoints.waypoints(temp(end), 1), waypoints.waypoints(temp(end), 2), 'bx', 'MarkerSize', 8, 'LineWidth', 2);
end

hold off;