package mdp.simulation.event;

import java.awt.Desktop;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.logging.Level;
import java.util.logging.Logger;
import mdp.Main;
import mdp.common.Vector2;
import mdp.communication.Translator;
import mdp.common.Direction;
import mdp.map.Descriptor;
import mdp.map.Map;
import mdp.map.WPObstacleState;
import mdp.map.WPSpecialState;
import mdp.robot.Robot;
import mdp.robot.RobotAction;
import mdp.robot.ImagePath;
import mdp.simulation.view.GridSquare;
import mdp.simulation.view.HexFrame;
import mdp.simulation.IGUIControllable;
import mdp.solver.exploration.ExplorationSolver;
import mdp.solver.exploration.Terminator;
import mdp.solver.shortestpath.AStarSolver;
import mdp.solver.shortestpath.AStarSolverResult;
import mdp.solver.shortestpath.AStarUtil;
import mdp.solver.shortestpath.SolveType;
import mdp.communication.Compiler;
import mdp.communication.SocketCommunicator;

public class EventHandler implements IHandleable {

    private IGUIControllable _gui;
    private Timer _shortestPathThread;
    private Thread _explorationThread;
    private static boolean _isShortestPath = true;
    private volatile boolean _callbackCalled;
    private Timer _timerThread;
    
    private String wayPoint;

    public static boolean isShortestPath() {
        return _isShortestPath;
    }

    public EventHandler(IGUIControllable gui) {
        _gui = gui;
        _gui.reset();

        // frame event
        _gui.getMainFrame().addWindowListener(_wrapWindowAdapter(GUIWindowEvent.OnClose));

        // obstacle event
        _gui.getMainFrame()
                .getMainPanel()
                .getGridPanel()
                .getGridContainer().setGridAdapter(_wrapMouseAdapter(GUIClickEvent.OnToggleObstacle));

        // descriptor control event
        _gui.getMainFrame()
                .getMainPanel()
                .getDescCtrlPanel()
                .getOpenDescBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnOpen));
        _gui.getMainFrame()
                .getMainPanel()
                .getDescCtrlPanel()
                .getSaveDescBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnSave));
        _gui.getMainFrame()
                .getMainPanel()
                .getDescCtrlPanel()
                .getGetHexBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnGetHex));
        _gui.getMainFrame()
                .getMainPanel()
                .getDescCtrlPanel();
//                .getCheckWebBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnCheckWeb));

        // run control event
        _gui.getMainFrame()
                .getMainPanel()
                .getRunCtrlPanel()
                .getExplorationBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnExploration));
        _gui.getMainFrame()
                .getMainPanel()
                .getRunCtrlPanel()
                .getToWayPointBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnToWayPoint));
        _gui.getMainFrame()
			.getMainPanel()
			.getRunCtrlPanel()
			.getTestWayPointBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnTestWayPoint));
        _gui.getMainFrame()
        	.getMainPanel()
        	.getRunCtrlPanel()
        	.getShortestPathWPBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnShortestPathWP));
        _gui.getMainFrame()
    	.getMainPanel()
    	.getRunCtrlPanel()
    	.getShortestPathBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnShortestPath));
        _gui.getMainFrame()
    		.getMainPanel()
    		.getRunCtrlPanel()
    		.getImagePathBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnImagePath));
//        _gui.getMainFrame()
//                .getMainPanel()
//                .getRunCtrlPanel()
//                .getCombinedBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnCombined));

        // run control event
        _gui.getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel()
                .getTermRoundCheckbox().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnToggleRound));
/*        _gui.getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel();
        		.getResetBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnReset)); */
        
        _gui.getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel()
                .getStopBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnStop));
/*        _gui.getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel();
        		.getRestartBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnRestart)); */
        
        // simulation/non-simulation control event
        _gui.getMainFrame()
                .getMainPanel()
                .getSimCtrlPanel()
                .getSimCheckBox().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnToggleSim));
        _gui.getMainFrame()
    		.getMainPanel()
    		.getSimCtrlPanel()
    		.getWayPoint().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnSetWayPoint));
        _gui.getMainFrame()
                .getMainPanel()
                .getSimCtrlPanel()
                .getConnectBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnConnectBtn));
        _gui.getMainFrame()
        .getMainPanel()
        .getSimCtrlPanel()
        .getTestBtn().addMouseListener(_wrapMouseAdapter(GUIClickEvent.OnTestBtn));
    }

    @Override
    public void resolveHandler(GUIClickEvent hdlr, MouseEvent e) {
        switch (hdlr) {
            case OnToggleObstacle:
                _onToggleObstacle(e);
                break;
            case OnOpen:
                _onOpenDesc(e);
                break;
            case OnSave:
                _onSaveDesc(e);
                break;
            case OnGetHex:
                _onGetHex(e);
                break;
            case OnReset:
            	_onReset(e);
//            case OnCheckWeb:
//                _onCheckWeb(e);
//                break;
            case OnExploration:
                _onExploration(e);
                break;
            case OnSetWayPoint:
                _onSetWayPoint(e);
                break;
            case OnTestWayPoint:
                _onTestWayPoint(e);
                break;
            case OnToWayPoint:
                _onToWayPoint(e);
                break;
            case OnShortestPathWP:
                _onShortestPathWP(e);
                break;
            case OnShortestPath:
                _onShortestPath(e);
                break;
            case OnCombined:
                _onCombined(e);
                break;
            case OnRestart:
                _onRestart(e);
                break;
            case OnStop:
                _onStop(e);
                break;
            case OnToggleRound:
                _onToggleRound(e);
                break;
            case OnToggleSim:
                _onToggleSim(e);
                break;
            case OnConnectBtn:
                _onConnect(e);
                break;
            case OnStartTimer:
                _onStartTimer();
                break;
            case OnStopTimer:
                _onStopTimer();
                break;
            case OnResetTimer:
                _onResetTimer();
                break;
            case OnTestBtn:
            	_onTestButton();
            case OnImagePath:
            	_onImagePath(e);
        }
    }
    
    private void _onImagePath(MouseEvent e) {
        
            System.out.println("ImageButton Pressed");
            
            List<Vector2> obstacleList = _gui.getMap().getObstacleList();
            System.out.println(obstacleList); 
            
            
            //this obstaclewaypointlist already takes into account the boundaries and 4 directions
            List<ArrayList<Vector2>> obswaypointList = _gui.getMap().generateObstacleWayPointList(obstacleList);
            System.out.println(obswaypointList);
            
            /*
            [(5, 11)]
            [[(3, 11), (5, 11)], [(7, 11), (5, 11)], [(5, 13), (5, 11)], [(5, 9), (5, 11)]]*/
            
            ///////////////Shortest Path to each waypoint using forloop///////////////////
            System.out.println("Starting Shortest Path XXX");
            _isShortestPath = true;
            
           // Vector2 obstaclewaypoint = new Vector2(3,11);
            //Vector2 obstaclewaypoint2 = new Vector2(11,11);
            
            try {
                int exePeriod = Integer.parseInt(
                        _gui.getMainFrame()
                                .getMainPanel()
                                .getRunCtrlPanel()
                                .getExePeriod().getText()
                );
           
            _shortestPathProcedureImage(exePeriod, obswaypointList);
        } catch (IOException e3) {
            // TODO Auto-generated catch block
            e3.printStackTrace();
        }    
         
    }
    
    //Look up gui for obstacle list after exploration
    	
    	
    	
        //Map map = ExplorationSolver.getMapViewer().getSubjectiveMap();
        //int[][] explored = ExplorationSolver.getMapViewer().getExplored();

        //String descStr = Descriptor.stringify(map, explored);
        //String content = String.join("\n", Descriptor.toHex(descStr));
        
        //System.out.println(Compiler.compileMap(map, explored));

       // HexFrame hexFrame = new HexFrame();
        //hexFrame
        //       .getHexFramePanel()
         //       .getHexTextArea().setText(content);

        //System.out.println("Get hex completed.");
		//return null ;
    //}
    
    
    

    private void _onTestButton(){
		Main.getRpi().sendSensingRequest();
		
	}

	private void _onSetWayPoint(MouseEvent e) {
		// TODO Auto-generated method stub
		wayPoint=Main.wayPoint;
		if(wayPoint.charAt(0) == 'w' && wayPoint.charAt(3) == 'p') {
        _gui
                .getMainFrame()
                .getMainPanel()
                .getSimCtrlPanel()
                .getWayPoint()
                .setText(wayPoint);}
		else
		{
			System.out.println("wrong format");
		}
		
	}

	@Override
    public void resolveFrameHandler(GUIWindowEvent hdlr, WindowEvent e) {
        switch (hdlr) {
            case OnClose:
                _onClose(e);
                break;
        }
    }

    private MouseAdapter _wrapMouseAdapter(GUIClickEvent hdlr) {
        return new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                resolveHandler(hdlr, e);
            }
        };

    }

    private WindowAdapter _wrapWindowAdapter(GUIWindowEvent hdlr) {
        return new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                resolveFrameHandler(hdlr, e);
            }
        };
    }

    private void _onClose(WindowEvent e) {
        System.exit(0);
    }

    private void _onSaveDesc(MouseEvent e) {
        try {
            String filePath = _gui.getMainFrame()
                    .getMainPanel()
                    .getDescCtrlPanel()
                    .getFilePathTextField().getText();
            /////// for testing
            boolean[][] explored = new boolean[Map.DIM_I][Map.DIM_J];
            for (int i = 0; i < Map.DIM_I; i++) {
                for (int j = 0; j < Map.DIM_J; j++) {
                    explored[i][j] = true;
                }
            }
            ///////
            Descriptor.saveToFile(filePath, _gui.getMap(), explored);
            System.out.println("Save desc completed.");
        } catch (IOException ex) {
            Logger.getLogger(IGUIControllable.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private void _onOpenDesc(MouseEvent e) {
        try {
            _gui.reset();
            String filePath = _gui.getMainFrame()
                    .getMainPanel()
                    .getDescCtrlPanel()
                    .getFilePathTextField().getText();
            System.out.println(filePath);
            _gui.update(Descriptor.parseFromFile(filePath)); //open test map txt file
            System.out.println("Open desc completed.");
        } catch (IOException ex) {
            Logger.getLogger(IGUIControllable.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private void _onGetHex(MouseEvent e) {
        Map map = ExplorationSolver.getMapViewer().getSubjectiveMap();
        int[][] explored = ExplorationSolver.getMapViewer().getExplored();

        String descStr = Descriptor.stringify(map, explored);
        String content = String.join("\n", Descriptor.toHex(descStr));
        
        System.out.println(Compiler.compileMap(map, explored));

        HexFrame hexFrame = new HexFrame();
        hexFrame
                .getHexFramePanel()
                .getHexTextArea().setText(content);

        System.out.println("Get hex completed.");
    }

/*    private void _onCheckWeb(MouseEvent e) {
        Map map = ExplorationSolver.getMapViewer().getSubjectiveMap();
        int[][] explored = ExplorationSolver.getMapViewer().getExplored();

        String descStr = Descriptor.stringify(map, explored);
        String[] content = Descriptor.toHex(descStr);

        String p1 = content[0];
        String p2 = content[1];
        String sampleArena = _gui
                .getMainFrame()
                .getMainPanel()
                .getDescCtrlPanel()
                .getFilePathTextField()
                .getText();

        try {
            Desktop.getDesktop().browse(new URI(
                    "http://mdpcx3004.sce.ntu.edu.sg/mapdescriptor.php?"
                    + "P1=" + p1 + "&"
                    + "P2=" + p2 + "&"
                    + "samplearena=" + sampleArena));
        } catch (URISyntaxException | IOException ex) {
            Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
        }
    } */
    
    
    //Not needed
	public Vector2 toVector(String waypoint) {
		// changes waypoint from string to vector
		String[] arrOfStr = waypoint.split(",", 2);
		int _i = Integer.parseInt(arrOfStr[0]);
		int _j = Integer.parseInt(arrOfStr[1]);
		Vector2 vector_waypoint = new Vector2(_i, _j);
		return vector_waypoint;
	
	}

    private void _onExploration(MouseEvent e) {
        int exePeriod = Integer.parseInt(
                _gui.getMainFrame()
                        .getMainPanel()
                        .getRunCtrlPanel()
                        .getExePeriod().getText()
        );
        _explorationThread = new Thread(() -> {
            try {
                _explorationProcedure(exePeriod, () -> {
                    _gui.trigger(GUIClickEvent.OnStopTimer);
                });
            } catch (InterruptedException ex) {
                Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
            } catch (IOException e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
            }
        });
        _explorationThread.start();
    }

    private void _onToWayPoint(MouseEvent e) {
        try {
            int exePeriod = Integer.parseInt(
                    _gui.getMainFrame()
                            .getMainPanel()
                            .getRunCtrlPanel()
                            .getExePeriod().getText()
            );
            _shortestPathProcedure(exePeriod);
        } catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
    }

   
      
    private void _onCombined(MouseEvent e) {
        int exePeriod = Integer.parseInt(
                _gui.getMainFrame()
                        .getMainPanel()
                        .getRunCtrlPanel()
                        .getExePeriod().getText()
        );
        _explorationThread = new Thread(() -> {
            try {
                // exploration
                _explorationProcedure(exePeriod, () -> {
                    System.out.println("Is at COMBINED callback");
                    // shortest path
                    try {
                        _shortestPathProcedure(exePeriod);
                    } catch (IOException e1) {
                        // TODO Auto-generated catch block
                        e1.printStackTrace();
                    }
                });
            } catch (InterruptedException ex) {
                Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
            } catch (IOException ex) {
                Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
            }
        });
        _explorationThread.start();
    }
    
    
    //OnshortestPath New, _shortestPathProcedure(exePeriod);
    private void _onShortestPathWP(MouseEvent e) {
        try {
            int exePeriod = Integer.parseInt(
                    _gui.getMainFrame()
                            .getMainPanel()
                            .getRunCtrlPanel()
                            .getExePeriod().getText()
                    
            );
            _shortestPathProcedure2(exePeriod);
            //_gui.trigger(GUIClickEvent.OnStopTimer); can't put it here as try will block
        } catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
    }
    
    private void _onShortestPath(MouseEvent e) {
        try {
            int exePeriod = Integer.parseInt(
                    _gui.getMainFrame()
                            .getMainPanel()
                            .getRunCtrlPanel()
                            .getExePeriod().getText()
            );
            _shortestPathProcedure(exePeriod);
        } catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
    }
    
    private void _onTestWayPoint(MouseEvent e) {
    	_gui.trigger(GUIClickEvent.OnSetWayPoint);
    } 
    
    


    private void  _onStop(MouseEvent e) {
        //return new MouseAdapter() {
        //    @Override
         //   public void mouseClicked(MouseEvent e) {
                if (_shortestPathThread != null) {
                    _shortestPathThread.cancel();
                }
                if (_explorationThread != null) {
                    _explorationThread.stop();
                }
                _gui.getMap().clearAllHighlight();
                _gui.getRobot().cleanBufferedActions();
                //For simulation, need to reload the current map
                
                _gui.update(_gui.getMap(), new Robot());
                _gui.trigger(GUIClickEvent.OnStopTimer);
                _gui.trigger(GUIClickEvent.OnResetTimer);
                
                

                System.out.println("Stop completed.");
            }
     //   };
  //  }

    private void _onReset(MouseEvent e) {
        //return new MouseAdapter() {
        //    @Override
       //     public void mouseClicked(MouseEvent e) {
    	if (_shortestPathThread != null) {
            _shortestPathThread.cancel();
        }
        if (_explorationThread != null) {
            _explorationThread.stop();
        }
        _gui.getMap().clearAllHighlight();
        _gui.getRobot().cleanBufferedActions();
        _gui.update(_gui.getMap(), new Robot());
        //For simulation, need to reload the current map
        _gui.trigger(GUIClickEvent.OnStopTimer);
        _gui.trigger(GUIClickEvent.OnResetTimer);
        _gui.reset();
        
                
                 
         System.out.println("Reset completed.");     
            }
     //   };
   // }
    
    private void _onRestart(MouseEvent e) {   
    	if (_shortestPathThread != null) {
            _shortestPathThread.cancel();
        }
        if (_explorationThread != null) {
            _explorationThread.stop();
        }
        
        _gui.getRobot().cleanBufferedActions();
        _gui.reset();
        _gui.getMainFrame().dispose();
        Main.startGUI();
        _isShortestPath = true;
        System.out.println("Restart completed.");
    }

    private void _onToggleObstacle(MouseEvent e) {
        GridSquare target = (GridSquare) e.getSource();
        Vector2 clickedPos = target.position();
        WPObstacleState obsState = _gui.getMap().getPoint(clickedPos).obstacleState();
        if (obsState.equals(WPObstacleState.IsActualObstacle)) {
            _gui.getMap().clearObstacle(clickedPos);
        } else {
            _gui.getMap().addObstacle(clickedPos);
        }
        target.toggleBackground();

        System.out.println("Toggled obstacle at " + clickedPos);
    }

    private void _onToggleRound(MouseEvent e) {
        boolean isChecked = _gui
                .getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel()
                .getTermRoundCheckbox().isSelected();
        if (isChecked) {
            System.out.println("Enabled terminating after 1st round");
        } else {
            System.out.println("Disabled terminating after 1st round");
        }
    }

    private void _onToggleSim(MouseEvent e) {
        boolean isSelected = _gui.getMainFrame()
                .getMainPanel()
                .getSimCtrlPanel()
                .getSimCheckBox().isSelected();
        Main.isSimulating(isSelected);
        if (isSelected) {
            System.out.println("Simulation mode enabled.");
        } else {
            System.out.println("Simulation mode disabled.");
        }
    }

    private void _onConnect(MouseEvent e) {
        try {
            Main.connectToRpi();
        } catch (IOException ex) {
            Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private void _onStartTimer() {
        Date startTime = new Date();
        System.out.println("startTime = " + startTime);
        _timerThread = new Timer();
        _timerThread.schedule(new TimerTask() {
            @Override
            public void run() {
                Date diffTime = new Date(new Date().getTime() - startTime.getTime() - 1800000);
                String timeStr = new SimpleDateFormat("mm:ss").format(diffTime);
                _gui.getMainFrame()
                        .getMainPanel()
                        .getStatsCtrlPanel()
                        .setTime(timeStr);
            }
        }, 1000, 1000);
    }

    private void _onStopTimer() {
        _timerThread.cancel();
    }
    
    private void _onResetTimer() {
    	_gui.getMainFrame()
        	.getMainPanel()
        	.getStatsCtrlPanel()
        	.setTime("00:00");
    }

    // shared procedures
    private void _explorationProcedure(int exePeriod, Runnable callback) throws InterruptedException, IOException {
        System.out.println("Starting Exploration");
        _isShortestPath = false;
        _callbackCalled = false;
        int termCoverage = Integer.parseInt(_gui
                .getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel()
                .getTermCoverageText().getText()
        );
        long termTime = Integer.parseInt(_gui
                .getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel()
                .getTermTimeText().getText()
        );
        boolean termRound = _gui
                .getMainFrame()
                .getMainPanel()
                .getIntrCtrlPanel()
                .getTermRoundCheckbox().isSelected();

        @SuppressWarnings("deprecation")
		Runnable interruptCallback = () -> {
            if (!_callbackCalled) {
                _callbackCalled = true;
                System.out.println(">> STOP <<");
                Robot curRobot = ExplorationSolver.getRobot();
                int[][] explored = ExplorationSolver.getMapViewer().getExplored();
                _explorationThread.stop();
                Map finalMap = new Map(explored, false);
                try {
                    ExplorationSolver.goBackToStart(finalMap, curRobot, callback);
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (InterruptedException ex) {
                    Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
                }
                Main.getGUI().update(finalMap);
            }
        };
        Runnable nonInterruptCallback = () -> {
            if (!_callbackCalled) {
                _callbackCalled = true;
                Robot curRobot = ExplorationSolver.getRobot();
                int[][] explored = ExplorationSolver.getMapViewer().getExplored();
                Map finalMap = new Map(explored, false);
                try {
                    ExplorationSolver.goBackToStart(finalMap, curRobot, callback);
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (InterruptedException ex) {
                    Logger.getLogger(EventHandler.class.getName()).log(Level.SEVERE, null, ex);
                }
                Main.getGUI().update(finalMap);
            }
        };

        Terminator terminator = null;
        if (termRound) {
            terminator = new Terminator(1, interruptCallback);
        } else if (termCoverage != 0 && termCoverage != 100) {
            terminator = new Terminator(termCoverage / 100f, interruptCallback);
        } else if (termTime != 0) {
        	System.out.println(termTime);
            terminator = new Terminator(termTime, interruptCallback);
        }
        if (terminator != null) {
            terminator.observe();
        }
        
        
        _gui.trigger(GUIClickEvent.OnStartTimer);
        ExplorationSolver.solve(_gui.getMap(), exePeriod);
        System.out.println("Exploration completed.");
        nonInterruptCallback.run();
    }

    private void _shortestPathProcedure(int exePeriod) throws IOException {
        System.out.println("Starting Shortest Path to waypoint");
        _isShortestPath = true;
        _gui.trigger(GUIClickEvent.OnStartTimer);
        
        String waypoint = _gui
                .getMainFrame()
                .getMainPanel()
                .getSimCtrlPanel()
                .getWayPoint()
                .getText();
        Vector2 vec_waypoint = new Vector2(waypoint);
        
        System.out.println("current waypoint = " + vec_waypoint);
        

        AStarSolver solver = new AStarSolver();
        Map map = _gui.getMap();
        Robot robot = _gui.getRobot();

        // decide two go for diagonal or not
        List<Vector2> normalSolve = solver.solve(map, robot, map.GOAL_POS).shortestPath;
        for (Vector2 curPos : normalSolve) {
        	System.out.println(curPos.toString());
        }
        List<Vector2> diagonalSolve = AStarUtil.smoothenPath(map, solver.solve(map, robot, map.GOAL_POS, SolveType.Smooth).shortestPath, false);
        float normalPathCost
                = 0.65f * AStarUtil.countTurn(normalSolve)
                + 0.35f * AStarUtil.calDistance(normalSolve);
        float diagPathCost
                = 0.65f * AStarUtil.countTurnSmooth(diagonalSolve)
                + 0.35f * AStarUtil.calDistance(diagonalSolve);

        System.out.println("normalPathCost to waypoint = " + normalPathCost);
        System.out.println("diagPathCost to waypoint = " + diagPathCost);

        if (false) {
    	//if (diagPathCost < normalPathCost) {
            // do diagonal
            System.out.println("smoothPath = ");
            diagonalSolve.forEach(pos -> {
                System.out.println(pos);
            });
            map.highlight(diagonalSolve, WPSpecialState.IsPathPoint);
            robot.position(diagonalSolve.get(diagonalSolve.size() - 1));
            _gui.update(map, robot);

            if (!Main.isSimulating()) {
                Main.getRpi().sendSmoothMoveCommand(diagonalSolve);
            }
        } else {
            // do quad directional
            map.highlight(normalSolve, WPSpecialState.IsPathPoint);
            LinkedList<RobotAction> actions = RobotAction
                    .fromPath(_gui.getRobot(), normalSolve);

            System.out.println("Main.isSimulating() = " + Main.isSimulating());
            if (!Main.isSimulating()) {
                // messaging arduino
                System.out.println("Sending sensing request to rpi (-> arduino) ");
                Main.getRpi().sendMoveCommand(actions, Translator.MODE_1);
            }
            _shortestPathThread = new Timer();
            _shortestPathThread.schedule(new TimerTask() {
                @Override
                public void run() {
                    if (!actions.isEmpty()) {
                        _gui.getRobot().execute(actions.pop());
                        _gui.update(_gui.getMap(), _gui.getRobot());
                    } else {
                        System.out.println("Path to waypoint completed.");
                        _gui.trigger(GUIClickEvent.OnStopTimer);
                        this.cancel();
                    }
                }
            }, exePeriod, exePeriod);
        }     
    }
    
    
    private void _shortestPathProcedureImage(int exePeriod, List<ArrayList<Vector2>> obswaypointList) throws IOException {
        System.out.println("Starting Shortest Path to all obstacle waypoint");
        _isShortestPath = true;
        _gui.trigger(GUIClickEvent.OnStartTimer);
        
        AStarSolver solver = new AStarSolver();
        Map map = _gui.getMap();
        Robot robot = _gui.getRobot();
        
        
        //First Obstacle
		Vector2 vec_waypoint = obswaypointList.get(0).get(0);
		Vector2 obstacle = obswaypointList.get(0).get(1);
		Direction desired_Orientation = Direction.getDesiredOrientation(vec_waypoint, obstacle);
		System.out.println("Desired Orientation = " + desired_Orientation);
	    Direction starting_direction = Direction.Right;  //Facing North
       
        // decide two go for diagonal or not
        List<Vector2> normalSolve = solver.solve2(map, new Vector2(1,1), vec_waypoint).shortestPath;
        map.highlight(normalSolve, WPSpecialState.IsPathPoint);
        
        //By default is facing North which is right
        //facing right is down //facing up is right
        //facing left is up //facing down is left
        ImagePath tempStore = RobotAction
        		.fromPath_SetPos(_gui.getRobot(), normalSolve, new Vector2(1,1), starting_direction);
        LinkedList<RobotAction> actions = tempStore.getActionList();		
        Direction cur_Orientation = tempStore.getCurrentOrientation();
        
        
        if(cur_Orientation != desired_Orientation)  // |Robot|   -> |OBS|
        {
        	if(desired_Orientation == Direction.Up)
        	{
        		if(cur_Orientation == Direction.Right)
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        		else if(cur_Orientation == Direction.Down)
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        		else if(cur_Orientation == Direction.Left)
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        	}
        	if(desired_Orientation == Direction.Down)    // |OBS|    <- |Robot|
        	{
        		if(cur_Orientation == Direction.Right) 
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        		if(cur_Orientation == Direction.Left) 
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Up) 
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        	}        	
        	if(desired_Orientation == Direction.Left)    //  |Robot|
        		//											  |OBS|
        	{
        		if(cur_Orientation == Direction.Up) 
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Right) 
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Down) 
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        	}
        	if(desired_Orientation == Direction.Right)    //    |OBS|
        		//											  |Robot|
        	{
        		if(cur_Orientation == Direction.Up) 
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        		if(cur_Orientation == Direction.Left) 
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Down) 
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        	}
        }
        
             
        
        //List<Vector2> listFinal= new ArrayList<Vector2>();
		//listFinal.addAll(normalSolve);
		//listFinal.addAll(normalSolve2);
/*		 for (Vector2 curPos : listFinal) {
			 System.out.println(curPos.toString());
		 } */
        
        Vector2 vec_waypoint2 = obswaypointList.get(1).get(0);
		Vector2 obstacle2 = obswaypointList.get(1).get(1);
		desired_Orientation = Direction.getDesiredOrientation(vec_waypoint2, obstacle2);
		System.out.println("Desired Orientation = " + desired_Orientation);
		Direction prev_direction = Direction.getDesiredOrientation(vec_waypoint, obstacle); //the prev iteration's desired_dir aka getdesiredorientation waypoint1
        List<Vector2> normalSolve2 = solver.solve2(map, vec_waypoint, vec_waypoint2).shortestPath;
        map.highlight(normalSolve2, WPSpecialState.IsPathPoint);
        ImagePath tempStore2 = RobotAction.fromPath_SetPos(_gui.getRobot(), normalSolve2,vec_waypoint, prev_direction);
        LinkedList<RobotAction> actions2 = tempStore2.getActionList();
        cur_Orientation = tempStore2.getCurrentOrientation();
        
        
        
        actions.addAll(actions2);
      
        
        if(cur_Orientation != desired_Orientation)  // |Robot|   -> |OBS|
        {
        	if(desired_Orientation == Direction.Up)
        	{
        		if(cur_Orientation == Direction.Right)
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        		else if(cur_Orientation == Direction.Down)
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        		else if(cur_Orientation == Direction.Left)
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        	}
        	if(desired_Orientation == Direction.Down)    // |OBS|    <- |Robot|
        	{
        		if(cur_Orientation == Direction.Right) 
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        		if(cur_Orientation == Direction.Left) 
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Up) 
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        	}        	
        	if(desired_Orientation == Direction.Left)    //  |Robot|
        		//											  |OBS|
        	{
        		if(cur_Orientation == Direction.Up) 
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Right) 
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Down) 
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        	}
        	if(desired_Orientation == Direction.Right)    //    |OBS|
        		//											  |Robot|
        	{
        		if(cur_Orientation == Direction.Up) 
        		{
        			actions.add(RobotAction.RotateRight);
        		}
        		if(cur_Orientation == Direction.Left) 
        		{
        			actions.add(RobotAction.RotateLeft);
        			actions.add(RobotAction.RotateLeft);
        		}
        		if(cur_Orientation == Direction.Down) 
        		{
        			actions.add(RobotAction.RotateLeft);
        		}
        	}
        }
        
        
    
        
        
        
        
        


       // System.out.println("Main.isSimulating() = " + Main.isSimulating());
            
            
        

        if (!Main.isSimulating()) {
                // messaging arduino
                System.out.println("Sending sensing request to rpi (-> arduino) ");
                Main.getRpi().sendMoveCommand(actions, Translator.MODE_1);
            }
            _shortestPathThread = new Timer();
            _shortestPathThread.schedule(new TimerTask() {
                @Override
                public void run() {
                    if (!actions.isEmpty()) {
                        _gui.getRobot().execute(actions.pop());
                        _gui.update(_gui.getMap(), _gui.getRobot());
                    } else {
                        System.out.println("Shortest path completed.");
                        _gui.trigger(GUIClickEvent.OnStopTimer);
                        this.cancel();
                    }
                }
            }, exePeriod, exePeriod);
        }	
        


    
    private void _shortestPathProcedure2(int exePeriod) throws IOException {
        System.out.println("Starting Shortest Path from Start Pos, to WP, to goal position");
        _isShortestPath = true;
        _gui.trigger(GUIClickEvent.OnStartTimer);
        
        String waypoint = _gui
                .getMainFrame()
                .getMainPanel()
                .getSimCtrlPanel()
                .getWayPoint()
                .getText();
        Vector2 vec_waypoint = new Vector2(waypoint);
        Vector2 goalpos = new Vector2(13,18); //temp goalpos
        
        System.out.println("current waypoint position = " + vec_waypoint);
        

        AStarSolver solver = new AStarSolver();
        Map map = _gui.getMap();
        Robot robot = _gui.getRobot();
        goalpos = map.GOAL_POS; //overwrite with actual goal
        
        

        // decide two go for diagonal or not
        List<Vector2> normalSolve = solver.solve(map, robot, vec_waypoint).shortestPath;
        List<Vector2> normalSolve2 = solver.solve2(map, vec_waypoint, goalpos).shortestPath;
        List<Vector2> listFinal= new ArrayList<Vector2>();
		listFinal.addAll(normalSolve);
		listFinal.addAll(normalSolve2);
/*		 for (Vector2 curPos : listFinal) {
			 System.out.println(curPos.toString());
		 } */
        //List<Vector2> diagonalSolve = AStarUtil.smoothenPath(map, solver.solve(map, robot, goalpos, SolveType.Smooth).shortestPath, false);
        //List<Vector2> normalSolve = solver.solve(map, robot).shortestPath;
        List<Vector2> diagonalSolve = AStarUtil.smoothenPath(map, solver.solve(map, robot, SolveType.Smooth).shortestPath, false);
        
        float normalPathCost
                = 0.65f * AStarUtil.countTurn(normalSolve)
                + 0.35f * AStarUtil.calDistance(normalSolve);
        float diagPathCost
                = 0.65f * AStarUtil.countTurnSmooth(diagonalSolve)
                + 0.35f * AStarUtil.calDistance(diagonalSolve);

        System.out.println("normalPathCost to goal = " + normalPathCost);
        System.out.println("diagPathCost to goal = " + diagPathCost);

        if (false) {
    	//if (diagPathCost < normalPathCost) {
            // do diagonal
            System.out.println("smoothPath = ");
            diagonalSolve.forEach(pos -> {
                System.out.println(pos);
            });
            map.highlight(diagonalSolve, WPSpecialState.IsPathPoint);
            robot.position(diagonalSolve.get(diagonalSolve.size() - 1));
            _gui.update(map, robot);

            if (!Main.isSimulating()) {
                Main.getRpi().sendSmoothMoveCommand(diagonalSolve);
            }
        } else {
            // do quad directional
            map.highlight(listFinal, WPSpecialState.IsPathPoint);
            LinkedList<RobotAction> actions = RobotAction
                    .fromPath(_gui.getRobot(), listFinal);

            System.out.println("Main.isSimulating() = " + Main.isSimulating());
            
            
            if (!Main.isSimulating()) {
                // messaging arduino
                System.out.println("Sending sensing request to rpi (-> arduino) ");
                Main.getRpi().sendMoveCommand(actions, Translator.MODE_1);
            }
            _shortestPathThread = new Timer();
            _shortestPathThread.schedule(new TimerTask() {
                @Override
                public void run() {
                    if (!actions.isEmpty()) {
                        _gui.getRobot().execute(actions.pop());
                        _gui.update(_gui.getMap(), _gui.getRobot());
                    } else {
                        System.out.println("Shortest path completed.");
                        _gui.trigger(GUIClickEvent.OnStopTimer);
                        this.cancel();
                    }
                }
            }, exePeriod, exePeriod);
        }	
        
    }
    	

        /////////////////////////////
//        AStarSolverResult solveResult = solver.solve(_gui.getMap(), _gui.getRobot());
//        _gui.getMap().highlight(solveResult.openedPoints, WPSpecialState.IsOpenedPoint);
//        _gui.getMap().highlight(solveResult.closedPoints, WPSpecialState.IsClosedPoint);
//        _gui.getMap().highlight(solveResult.shortestPath, WPSpecialState.IsPathPoint);
//        LinkedList<RobotAction> actions = RobotAction
//                .fromPath(_gui.getRobot(), solveResult.shortestPath);
//
//        System.out.println("Main.isSimulating() = " + Main.isSimulating());
//        if (!Main.isSimulating()) {
//            // messaging arduino
//            System.out.println("Sending sensing request to rpi (-> arduino) ");
//            Main.getRpi().sendMoveCommand(actions, Translator.MODE_1);
//        }
//        _shortestPathThread = new Timer();
//        _shortestPathThread.schedule(new TimerTask() {
//            @Override
//            public void run() {
//                if (!actions.isEmpty()) {
//                    _gui.getRobot().execute(actions.pop());
//                    _gui.update(_gui.getMap(), _gui.getRobot());
//                } else {
//                    System.out.println("Shortest path completed.");
//                    this.cancel();
//                }
//            }
//        }, exePeriod, exePeriod);
        /////////////////////////////
//        AStarSolverResult solveResult = solver.solve(map, robot, SolveType.Smooth);
//        List<Vector2> smoothPath = AStarUtil.smoothenPath(map, solveResult.shortestPath, false);
//        System.out.println("smoothPath = ");
//        smoothPath.forEach(pos -> {
//            System.out.println(pos);
//        });
//        map.highlight(smoothPath, WPSpecialState.IsPathPoint);
//        robot.position(smoothPath.get(smoothPath.size() - 1));
//        _gui.update(map, robot);
//
//        if (!Main.isSimulating()) {
//            Main.getRpi().sendSmoothMoveCommand(smoothPath);
//        }
        /////////////////////////////
    	
    }
