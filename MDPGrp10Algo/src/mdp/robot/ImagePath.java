package mdp.robot;

import java.io.IOException;

import java.util.Arrays;
import java.util.LinkedList;

import mdp.common.Direction;
import mdp.common.Vector2;
import mdp.communication.Translator;
import mdp.solver.exploration.ActionFormulator;
import mdp.solver.exploration.ExplorationSolver;
import mdp.solver.exploration.MapViewer;
import mdp.solver.exploration.SensingData;
import mdp.Main;
import mdp.map.Map;

public class ImagePath {
	private LinkedList<RobotAction> _actions;
	private Direction _current_orientation;
	
	public ImagePath (LinkedList<RobotAction> actions, Direction current_orientation) {
        _actions = actions;
        _current_orientation = current_orientation;
    }
	
	public LinkedList<RobotAction> getActionList()
	{
		return _actions;
	}
	
	public Direction getCurrentOrientation()
	{
		return _current_orientation;
	}
	

}
