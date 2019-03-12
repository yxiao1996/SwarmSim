classdef Circle < obstacle
    %CIRCLE 
    % circle obstacle in the simulated environment
    % InObstacle - whether point in obstacle
    % InBoundBox - whether point in bounding box
    properties
        radius
    end
    
    methods
        function obj = Circle(center,radius)
            %CIRCLE 
            obj.position = center;
            obj.radius = radius;
            obj.type = "circle";
            % calculate bounding box for the obstacle
            c_x = center(1); c_y = center(2);
            pad = 0.05;
            r_pad = radius + pad;
            left = c_x - r_pad; right = c_x + r_pad;
            top = c_y + r_pad; bottom = c_y - r_pad;
            obj.bound_box = [left top right bottom];
        end
        
        function in_area = InBoundBox(obj,xy)
            % determine whether point xy in bounding box
            x = xy(1); y = xy(2);
            l = obj.bound_box(1);
            t = obj.bound_box(2);
            r = obj.bound_box(3);
            b = obj.bound_box(4);
            if ((x > l && x < r) && (y > b && y < t))
                in_area = true;
            else
                in_area = false;
            end
        end
        
        function in_area = InObstacle(obj,xy)
            % determine whether point xy in obstacle area
            x = xy(1); y = xy(2);
            x_c = obj.position(1); y_c = obj.position(2);
            dist = sqrt((x-x_c)^2+(y-y_c)^2);
            if (dist < obj.radius)
                in_area = true;
            else
                in_area = false;
            end
        end
    end
end

