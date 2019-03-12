classdef AABB
    %AABB physics engine to detect collision between robot and environment
    %   此处显示详细说明
    
    properties
        map
        ofs
        numRobots
        radius
        step_x
        step_y
        force
        report
    end
    
    methods
        function obj = AABB(map,numRobots,r,report)
            obj.map = map; % the map of the environment
            obj.ofs = [
                [-r;r;0], [r;r;0], [r;-r;0], [-r;-r;0],...
                [0;r;0], [r;0;0],[0;-r;0],[-r;0;0]
            ];
            obj.numRobots = numRobots;
            obj.radius = r;
            map_range_x = map.XWorldLimits(2)-map.XWorldLimits(1);
            map_range_y = map.YWorldLimits(2)-map.YWorldLimits(1);
            obj.step_x = map_range_x / map.GridSize(2);
            obj.step_y = map_range_y / map.GridSize(1);
            obj.report = report;
        end
        
        function new_poses = check_robots(obj,poses,prev_poses)
            new_poses = zeros(size(poses));
            % check if any pair of robots collide
            for i = 1:obj.numRobots
                for j = i+1:obj.numRobots
                    xy1 = poses(1:2,i);
                    xy2 = poses(1:2,j);
                    pose1 = zeros(3,1);
                    pose2 = zeros(3,1);
                    dist = sqrt(dot(xy1-xy2,xy1-xy2));
                    %disp(dist);
                    if (dist < obj.radius*2) % two robot collide
                        if (obj.report)
                            disp("robot collision!");
                        end
                        d = (obj.radius*2 - dist)/2;
                        e = (xy2 - xy1)./dist;
                        %disp(e);
                        pose1(1:2) = xy1 - e.*d;%prev_poses(1:2,i);%
                        pose1(3) = poses(3,i);
                        pose2(1:2) = xy2 + e.*d;%prev_poses(1:2,j);%
                        pose2(3) = poses(3,j);
                    else
                        pose1 = poses(:,i);
                        pose2 = poses(:,j);
                    end
                    new_poses(:,i) = pose1;
                    new_poses(:,j) = pose2;
                end
            end
        end
        
        function new_poses = check_obstacles(obj,poses,prev_poses)
            new_poses = zeros(size(poses));
            % loop through all poses and check collisions
            for i = 1:obj.numRobots
                pose = poses(:,i);
                prev_pose = prev_poses(:,i);
                new_pose = prev_pose;
                verts = [pose pose pose pose pose pose pose pose] + obj.ofs;
                occupy = checkOccupancy(obj.map,verts(1:2,:)');
                action = zeros(3,1);
                %for j = 1:4
                %    if(occupy(j) == 1) % vertex collide with wall
                %        action = action - obj.ofs(:,j).*[obj.step_x obj.step_y 0]'./obj.radius;
                %    end
                if (sum(occupy)>0)
                    A = occupy(1); B = occupy(2); C = occupy(3); D = occupy(4); 
                    E = occupy(5); F = occupy(6); G = occupy(7); H = occupy(8);
                    if (sum(occupy) == 2 || sum(occupy) == 3)
                        if((A+B==2 || C+D==2) ||...
                           (A+E==2 || B+E==2) ||...
                           (C+G==2 || D+G==2) ||...
                           (A+B+E==3)||(C+D+G==3))    
                            new_pose(1) = pose(1);
                            new_pose(2) = prev_pose(2);
                            new_pose(3) = pose(3);
                        elseif((B+C==2 || A+D==2) ||...
                               (B+F==2 || C+F==2) ||...
                               (A+H==2 || D+H==2) ||...
                               (B+C+F==3)||(A+D+H==3))
                            new_pose(1) = prev_pose(1);
                            new_pose(2) = pose(2);
                            new_pose(3) = pose(3);
                        else
                            new_pose = prev_pose;
                            new_pose(3) = pose(3);
                        end
                    elseif (sum(occupy) == 1)
                        %new_pose = prev_pose;
                        new_pose = pose;
                    else
                        new_pose = prev_pose;
                        new_pose(3) = pose(3);
                    end
                    new_poses(:,i) = new_pose;
                    if(obj.report)
                        disp("wall collision!");
                    end
                else
                    new_poses(:,i) = pose;
                end
            end
        end
    end
end

