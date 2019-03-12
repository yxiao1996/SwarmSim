classdef Triangle < obstacle
    %TRIANGLE 
    % triangle obstacle in the simulated environment
    
    properties
        points
        norm
        units
    end
    
    methods
        function obj = Triangle(center,radius,orient)
            %TRIANGLE 
            obj.position = center;
            obj.orientation = orient;
            obj.type = "triangle";
            % compute the apexed of a triangle
            R = [
                cos(orient) -sin(orient);
                sin(orient)  cos(orient)
            ];
            points = [
              1 0;
              -0.5 sqrt(3)/2;
              -0.5 -sqrt(3)/2
                ]*radius;
            points = points*R;
            obj.points = points;
            obj.norm = radius*sqrt(3)/2;
            % calculate bounding box for the obstacle
            c_x = center(1); c_y = center(2);
            pad = 0.05;
            r_pad = radius + pad;
            left = c_x - r_pad; right = c_x + r_pad;
            top = c_y + r_pad; bottom = c_y - r_pad;
            obj.bound_box = [left top right bottom];
            % calculating orthorgonal unit vectors of edges
            v1 = points(1,:)-points(2,:);
            v1 = [-v1(2) v1(1)];
            v1 = v1/sqrt(dot(v1,v1));
            
            v2 = points(2,:)-points(3,:);
            v2 = [-v2(2) v2(1)];
            v2 = v2/sqrt(dot(v2,v2));
            
            v3 = points(3,:)-points(1,:);
            v3 = [-v3(2) v3(1)];
            v3 = v3/sqrt(dot(v3,v3));
            obj.units = [v1;v2;v3];
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
            in_area = true;
            x = xy(1); y = xy(2);
            x_c = obj.position(1); y_c = obj.position(2);
            v = [x-x_c y-y_c];
            for i = 1:3
                e = obj.units(i,:);
                if(dot(v,e) > obj.norm)
                    in_area = false;
                end
            end
        end
    end
end

