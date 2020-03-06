package mdp.communication;

import java.io.IOException;
import java.sql.Date;
import java.util.ArrayList;
import java.util.LinkedList;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.logging.Level;
import java.util.logging.Logger;
import mdp.common.Vector2;
import mdp.map.Map;
import mdp.robot.RobotAction;
import mdp.solver.exploration.CalibrationType;

public class Translator implements ITranslatable {

    private static final String _TO_ARDUINO_MARKER = "a";
    private static final String _TO_ANDROID_MARKER = "b";
    
    private static final String _EXPLORATION_END_MARKER = "q";

    private static final String _MSG_SEPARATOR = "_";

    private static final int _PROBING_PERIOD = 20;
    
    public static final String MODE_0 = "";
    public static final String MODE_1 = "m1";
    public static final String MODE_2 = "m2";

    private SocketCommunicator _socketCommunicator;
    private String _inputBuffer;

    public Translator() throws IOException {
        _inputBuffer = "";
    }

    @Override
    public void connect(Runnable callback) {
        _socketCommunicator = new SocketCommunicator(callback);
    }

    @Override
    public String getInputBuffer() {
        return _inputBuffer;
    }

    @Override
    public void sendCalibrationCommand(CalibrationType calType) {
        try {
            String message = _TO_ARDUINO_MARKER + Compiler.compileCalibration(calType);
            _socketCommunicator.echo(message);
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    // Arduino
    @Override
    public void sendMoveCommand(List<RobotAction> actions, String mode) {
        try {
            String message = _TO_ARDUINO_MARKER + Compiler.compileActions(actions, mode);
            System.out.println("message = " + message);
            _socketCommunicator.echo(message);
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    public void sendShortestPathMoveCommand(String actions, String mode) {
        try {
            String message = _TO_ARDUINO_MARKER + actions;
            System.out.println("message = " + message);
            _socketCommunicator.echo(message);
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    public void disableDelay() {
    	
        try {
        	String message = _TO_ARDUINO_MARKER + "d";
        	System.out.println("message = " + message);
			_socketCommunicator.echo(message);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }

    @Override
    public void sendSmoothMoveCommand(List<Vector2> smoothPath) {
        try {
            String message = _TO_ARDUINO_MARKER + Compiler.compileSmoothActions(smoothPath, MODE_2);
            System.out.println("message = " + message);
            _socketCommunicator.echo(message);
//            _socketCommunicator.echo(_TO_ARDUINO_MARKER + "r54.321|");
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @Override
    public void sendSensingRequest() {
        try {
            String message = _TO_ARDUINO_MARKER + Compiler.compileSensingRequest();
            _socketCommunicator.echo(message);
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    // Android
    @Override
    public void sendInfoToAndroid(Map map, int[][] explored, LinkedList<RobotAction> actions) {
        try {
            String message = _TO_ANDROID_MARKER
                    + Compiler.compileMap(map, explored)
                    +","
                    + "\"robot\":"
                    +"\""
                    + Compiler.compileAndroidActions(actions)
                    +"\""
                    +"}";
            
            _socketCommunicator.echo(message);
            
            
            
//            List<String> arrActions=Compiler.compileAndroidActions(actions);
            
//            String action = _TO_ANDROID_MARKER+ "{"
//            		+ "\"robot\":"
//                  + "\"" + "rffflfff"
//                  + Compiler.compileActions(actions,MODE_0)
//                  +"\""+ "}";
            
//            _socketCommunicator.echo(action);
            
//            List<String> aActions= new ArrayList<String>();
//            aActions.add("f");
//            aActions.add("f");
//            aActions.add("r");
//            aActions.add("f");
//            aActions.add("f");
//            aActions.add("l");
            
//            for(String action:aActions) {
            	
//            	this.androidActions(action);
//            }
//                    + ",\"robot\":"
//                    + "\"" + "rffflfff"
//                    //+ Compiler.compileAndroidActions(actions)
//                    +"\""+ "}";
            
            
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    public void androidActions(String robotAction) {
    	try {
    		String a;
        	a = _TO_ANDROID_MARKER
        			+"{\"robot\" : \""
        			+robotAction+
        			"\""+ "}";
        	_socketCommunicator.echo(a);
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private String _readAsString() {
        try {
            return _socketCommunicator.read();
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
        return "ERRR";
    }

    @Override
    public void listen(Runnable handler) {
        new Thread() {
            @Override
            public void run() {
                Timer probingTimer = new Timer();
                probingTimer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        System.out.println("listening...");
                        String readResult = _readAsString();
                        _inputBuffer = "";
                        if (!readResult.isEmpty()) {
                            _inputBuffer = readResult;
                            handler.run();
                            System.out.println();
//                                probingTimer.cancel();
                        }
                    }
                }, 0, _PROBING_PERIOD);
            }
        }.start();
    }

    @Override
    public void sendExplorationEndMarker() {
        try {
            String message = _TO_ARDUINO_MARKER + 
                    Compiler.compileArbitrary(_EXPLORATION_END_MARKER);
            _socketCommunicator.echo(message);
        } catch (IOException ex) {
            Logger.getLogger(Translator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    public SocketCommunicator getSocketCommunicator() {
    	return _socketCommunicator;
    }
}
