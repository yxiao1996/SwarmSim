function ui()

    sim = createSimulation();
    gui = createInterface(sim.SimNames);
    curSim = evalin('base','initSimulation');
    runFlag = false;
    
    function onListSelection(src,~)
        sim.SelectedSim = src.Value;
    end

    function onSimStart(~,~)
        runFlag = true;
        while(runFlag)
            curSim = curSim.step();
            axis([0 15 0 15])
            redrawSimulation()
        end
    end

    function onSimStop(~,~)
        runFlag = false;
    end

    function simulation = createSimulation()
        % Create the shared data-structure for this application
        simList = {
            'VirtualStructure' 'VirtualStructureSimulation'
            'LeaderFollower'   'LeaderFollowerSimulation'
            'BehaviorBased'    'BehaviorBasedSimulation'
            };
        selectedSim = 1;
        simulation = struct(...
            'SimNames',{simList(:,1)'},...
            'SimClasses',{simList(:,2)'},...
            'SelectedSim',selectedSim);
    end

    function new_sim = initSimulation()
        % initializa a new simulation
        SimName = sim.SimClasses(sim.SelectedSim);
        
        % generate map for the simulation
        size = 15;
        resolution = 10;
        numObstacles = 0;
        space = 5;
        %p = zeros(size*resolution);
        map_gen = MapGenerate(size,size,space,resolution);
        [occupy_mat,map_gen] = map_gen.addBounds(2);
        for idx_obst = 1:numObstacles
            [occupy_mat,map_gen] = map_gen.addRandomObstacle(1.5,0.5);
        end
        map = robotics.OccupancyGrid(occupy_mat,resolution);
        %assignin('base','map',map);
        form = TriangleFormation;%DiamondFormation();
        numRobots = form.numRobots;
        numSensors = 5;
        sensorRange = 2.5;
        showTraj = false;
        initial_poses = 8*(rand(3,numRobots).*[0.5;0.5;0]) + [0.5;0.5;0];
        robotInfos = cell(1,numRobots);
        for i = 1:numRobots
            t = "DiffDrive"; % differential drive dynamics
            R = 0.1; 
            L = 0.5;
            s = numSensors;
            r = sensorRange;
            show = showTraj;
            robotInfo = RobotInfo(t,R,L,s,r);
            robotInfos{i} = robotInfo;
        end
        swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);
        new_sim = LeaderFollowerSimulation(map,swarmInfo,form);        
    end
    function gui = createInterface( demoList )
        % Create the user interface for the application and return a
        % structure of handles for global use.
        gui = struct();
        % Open a window and add some menus
        gui.Window = figure( ...
            'Name', 'Gallery browser', ...
            'NumberTitle', 'off', ...
            'MenuBar', 'none', ...
            'Toolbar', 'none', ...
            'HandleVisibility', 'off' );

        % Set default panel color
        %uiextras.set( gui.Window, 'DefaultBoxPanelTitleColor', [0.7 1.0 0.7] );

        % + File menu
        gui.FileMenu = uimenu( gui.Window, 'Label', 'File' );
        uimenu( gui.FileMenu, 'Label', 'Exit', 'Callback', @onExit );

        % + View menu
        gui.ViewMenu = uimenu( gui.Window, 'Label', 'View' );
        for ii=1:numel( demoList )
            uimenu( gui.ViewMenu, 'Label', demoList{ii}, 'Callback', @onMenuSelection );
        end

        % + Help menu
        helpMenu = uimenu( gui.Window, 'Label', 'Help' );
        uimenu( helpMenu, 'Label', 'Documentation', 'Callback', @onHelp );


        % Arrange the main interface
        mainLayout = uiextras.HBoxFlex( 'Parent', gui.Window, 'Spacing', 3 );

        % + Create the panels
        controlPanel = uiextras.BoxPanel( ...
            'Parent', mainLayout, ...
            'Title', 'Select a demo:' );
        gui.ViewPanel = uiextras.BoxPanel( ...
            'Parent', mainLayout, ...
            'Title', 'Viewing: ???', ...
            'HelpFcn', @onDemoHelp );

        % + Adjust the main layout
        set( mainLayout, 'Sizes', [-1,-2]  );


        % + Create the controls
        controlLayout = uiextras.VBox( 'Parent', controlPanel, ...
            'Padding', 3, 'Spacing', 3 );
        gui.ListBox = uicontrol( 'Style', 'list', ...
            'BackgroundColor', 'w', ...
            'Parent', controlLayout, ...
            'String', demoList(:), ...
            'Value', 1, ...
            'Callback', @onListSelection);
        gui.StartButton = uicontrol( 'Style', 'PushButton', ...
            'Parent', controlLayout, ...
            'String', 'Start Simulation', ...
            'Callback', @onSimStart );
        gui.StopButton = uicontrol( 'Style', 'PushButton',...
            'Parent', controlLayout, ...
            'String', 'Stop Simulation', ...
            'Callback', @onSimStop );
        %set( controlLayout, 'Sizes', [-1 24] ); % Make the list fill the space

        % + Create the view
        p = gui.ViewPanel;
        gui.ViewAxes = axes( 'Parent', p ,'OuterPosition',[0 0 15 15]);

    end % createInterface

    function redrawSimulation()
        % We first clear the existing axes ready to build a new one
        if ishandle( gui.ViewAxes )
            delete( gui.ViewAxes );
        end

        gui.ViewAxes = gca();
        fig = gcf();
        set(fig,'Visible','off');
        % Now copy the axes from the demo into our window and restore its
        % state.
        cmap = colormap( gui.ViewAxes );
        set( gui.ViewAxes, 'Parent', double(gui.ViewPanel) );
        colormap( gui.ViewAxes, cmap );
        rotate3d( gui.ViewAxes, 'on' );
        % Get rid of the demo figure
        close( fig )

    end
end

