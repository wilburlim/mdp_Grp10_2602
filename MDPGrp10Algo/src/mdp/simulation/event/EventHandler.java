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
import mdp.map.Descriptor;
import mdp.map.Map;
import mdp.map.WPObstacleState;
import mdp.map.WPSpecialState;
import mdp.robot.Robot;
import mdp.robot.RobotAction;
import mdp.simulation.view.GridSquare;
import mdp.simulation.view.HexFrame;
import mdp.simulation.IGUIControllable;
import mdp.solver.exploration.CalibrationType;
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
        }
    }

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
            	Main.getRpi().disableDelay();
                System.out.println("Sending sensing request to rpi (-> arduino) ");
                String _MOVE_FORWARD = "f";
                String _MOVE_BACKWARD = "b";
                String _ROTATE_LEFT = "l";
                String _ROTATE_RIGHT = "r";
                String _TRAILER = "|";
                String result = "";
                
                int count = 0;
                String lastAction = "";
                for (RobotAction action : actions) {
                    String nextActionStr;
                    switch (action) {
                        case MoveForward:
                            nextActionStr = _MOVE_FORWARD;
                            break;
                        case MoveBackward:
                            nextActionStr = _MOVE_BACKWARD;
                            break;
                        case RotateLeft:
                            nextActionStr = _ROTATE_LEFT;
                            break;
                        case RotateRight:
                            nextActionStr = _ROTATE_RIGHT;
                            break;
                        default:
                            nextActionStr = " ";
                            break;
                    }
                    if (result.length() != 0) {
                        if (lastAction.equals(nextActionStr)) {
                            boolean isRotating = lastAction.equals(_ROTATE_LEFT) || lastAction.equals(_ROTATE_RIGHT);
                            if (isRotating) {
                                result += _TRAILER + lastAction;
                            } else {
                                count++;
                            }
                        } else {
                            boolean isRotating = lastAction.equals(_ROTATE_LEFT) || lastAction.equals(_ROTATE_RIGHT);
                            result += (isRotating ? "" : count) + _TRAILER + nextActionStr;
                            count = 0;
                        }
                    } else {
                        result += nextActionStr;
                    }
                    lastAction = nextActionStr;
                }
                boolean isRotating;
                if (result.length() != 1) {
                    isRotating = lastAction.equals(_ROTATE_LEFT) || lastAction.equals(_ROTATE_RIGHT);
                } else {
                    isRotating = result.equals(_ROTATE_LEFT) || result.equals(_ROTATE_RIGHT);
                }

                result += (isRotating ? "" : count) + _TRAILER;
                System.out.println("Sending out shortest path: " + result);

                Main.getRpi().sendShortestPathMoveCommand(result, Translator.MODE_0);
                
                
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

                        if(!Main.isSimulating()) {
                        	Main.getRpi().sendCalibrationCommand(CalibrationType.Front);
                        }
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
