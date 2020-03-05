package mdp.map;

import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;
import mdp.common.Console;
import mdp.robot.Robot;
import mdp.common.Vector2;


public class Map {
    // constants
    public static final int DIM_I = 15;
    public static final int DIM_J = 20;
    public static final Vector2 START_POS = new Vector2(1, 1);
    public static final Vector2 GOAL_POS = new Vector2(DIM_I - 2, DIM_J - 2);
    public static final Vector2 WAYPOINT_POS = new Vector2(11, 11);
    
    private final Waypoint[][] _wpMap;
    
    public Map() {
        _wpMap = new Waypoint[DIM_I][DIM_J];  //declare an array of way points with all null way point objects
        for (int i = 0; i < DIM_I; i++) {  
            for (int j = 0; j < DIM_J; j++) {
                // init default values
                Vector2 curPos = new Vector2(i, j); //iterates for the entire map size
                WPSpecialState curSpecState = WPSpecialState.NA;
                WPObstacleState curObsState = WPObstacleState.IsWalkable;
                
                // change special state if applicable
                if (START_POS.equals(curPos)) {
                    curSpecState = WPSpecialState.IsStart;
                } else if (GOAL_POS.equals(curPos)) {
                    curSpecState = WPSpecialState.IsGoal;
                } else if (i == 0 || j == 0 || i == DIM_I - 1 || j == DIM_J - 1) {
                    curObsState = WPObstacleState.IsVirtualObstacle; //border squares of grid container
                }
                
                // create point
                _wpMap[i][j] = new Waypoint(curPos, curSpecState, curObsState);
            }
        }
    }
    
    public Map(int[][] explored, boolean isStillExploring) {
        _wpMap = new Waypoint[DIM_I][DIM_J];
        List<Vector2> obstacles = new ArrayList<>();
        for (int i = 0; i < DIM_I; i++) {
            for (int j = 0; j < DIM_J; j++) {
                // init default values
                Vector2 curPos = new Vector2(i, j);
                WPSpecialState curSpecState = WPSpecialState.NA;
                WPObstacleState curObsState = WPObstacleState.IsWalkable;
                
                // change special state if applicable
                if (START_POS.equals(curPos)) {
                    curSpecState = WPSpecialState.IsStart;
                } else if (GOAL_POS.equals(curPos)) {
                    curSpecState = WPSpecialState.IsGoal;
                } else {
                    switch (explored[i][j]) { //content in array, 1,2 or 0
                        case 0:
                        		if (!isStillExploring) {
                             	obstacles.add(curPos);
                        		}
                            break;
                        case 2:
                            obstacles.add(curPos);
                            curSpecState = WPSpecialState.IsExplored;
                            break;
                        case 1:
                            curSpecState = WPSpecialState.IsExplored;
                            break;
                        
                    }
                }
                
                
                if(i == 0 || i== DIM_I-1 || j == 0 || j==DIM_J-1)
                		curObsState = WPObstacleState.IsVirtualObstacle;
                // create point
                _wpMap[i][j] = new Waypoint(curPos, curSpecState, curObsState);
            }
        }
        addObstacle(obstacles);
    }
    
    private void _setObstacle(Vector2 pos, WPObstacleState obsState) {
        _wpMap[pos.i()][pos.j()].obstacleState(obsState);
    }
    
    public List<Vector2> getObstacleList() {
    	List<Vector2> obstacleList = new ArrayList<Vector2>();
    	for (int i = 0; i < DIM_I; i++) {  
            for (int j = 0; j < DIM_J; j++) {
            	if(_wpMap[i][j].obstacleState() ==  WPObstacleState.IsActualObstacle)
            	{
            		Vector2 cur_vector = new Vector2(i,j);
            		obstacleList.add(cur_vector);	
            	}
            	
            	
            }
            
        }
    	return obstacleList;
    	
    }
    
    public List<ArrayList<Vector2>> generateObstacleWayPointList(List<Vector2> obstacleList){
    	List<ArrayList<Vector2>> listOfLists = new ArrayList<ArrayList<Vector2>>();
    	//for each obstacle identified in the obstacle list in map from getObstacleList()
    	for (Vector2 obstacle_vector : obstacleList) {
    		
    		
    		ArrayList<Vector2> left_list = new ArrayList<Vector2>();
    		ArrayList<Vector2> right_list = new ArrayList<Vector2>();
    		ArrayList<Vector2> top_list = new ArrayList<Vector2>();
    		ArrayList<Vector2> btm_list = new ArrayList<Vector2>();
    		
    		//create the vectors up to 2 grids NSEW from the obstacle grid
    		
    		Vector2 obstacle_vector_left_1 = new Vector2(obstacle_vector.i() -1, obstacle_vector.j());
    		Vector2 obstacle_vector_left_2 = new Vector2(obstacle_vector.i() -2, obstacle_vector.j());
    		Vector2 obstacle_vector_right_1 = new Vector2(obstacle_vector.i()+1, obstacle_vector.j());
    		Vector2 obstacle_vector_right_2 = new Vector2(obstacle_vector.i()+2, obstacle_vector.j());
    		Vector2 obstacle_vector_top_1 = new Vector2(obstacle_vector.i(), obstacle_vector.j()+1);
    		Vector2 obstacle_vector_top_2 = new Vector2(obstacle_vector.i(), obstacle_vector.j()+2);
    		Vector2 obstacle_vector_btm_1 = new Vector2(obstacle_vector.i(), obstacle_vector.j()-1);
    		Vector2 obstacle_vector_btm_2 = new Vector2(obstacle_vector.i(), obstacle_vector.j()-2);
    		
    		
    		//North of Obstacle
    		Vector2 obstacle_vector_top_TL = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()+3);
    		Vector2 obstacle_vector_top_TM = new Vector2(obstacle_vector.i(), obstacle_vector.j()+3);
    		Vector2 obstacle_vector_top_TR = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()+3);
    		Vector2 obstacle_vector_top_ML = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()+2);
    		Vector2 obstacle_vector_top_MM = new Vector2(obstacle_vector.i(), obstacle_vector.j()+2);
    		Vector2 obstacle_vector_top_MR = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()+2);
    		Vector2 obstacle_vector_top_BL = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_top_BM = new Vector2(obstacle_vector.i(), obstacle_vector.j()+1);
    		Vector2 obstacle_vector_top_BR = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()+1);
    		List<Vector2> TopGrids = new ArrayList<Vector2>();
    		TopGrids.add(obstacle_vector_top_TL);
    		TopGrids.add(obstacle_vector_top_TM);
    		TopGrids.add(obstacle_vector_top_TR);
    		TopGrids.add(obstacle_vector_top_ML);
    		TopGrids.add(obstacle_vector_top_MM);
    		TopGrids.add(obstacle_vector_top_MR);
    		TopGrids.add(obstacle_vector_top_BL);
    		TopGrids.add(obstacle_vector_top_BM);
    		TopGrids.add(obstacle_vector_top_BR);
    		//.out.println(TopGrids.get(0));
    		
    		
    		//South of Obstacle
    		Vector2 obstacle_vector_btm_TL = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()-1);
    		Vector2 obstacle_vector_btm_TM = new Vector2(obstacle_vector.i(), obstacle_vector.j()-1);
    		Vector2 obstacle_vector_btm_TR = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()-1);
    		Vector2 obstacle_vector_btm_ML = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()-2);
    		Vector2 obstacle_vector_btm_MM = new Vector2(obstacle_vector.i(), obstacle_vector.j()-2);
    		Vector2 obstacle_vector_btm_MR = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()-2);
    		Vector2 obstacle_vector_btm_BL = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()-3);
    		Vector2 obstacle_vector_btm_BM = new Vector2(obstacle_vector.i(), obstacle_vector.j()-3);
    		Vector2 obstacle_vector_btm_BR = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()-3);
    		List<Vector2> BottomGrids = new ArrayList<Vector2>();
    		BottomGrids.add(obstacle_vector_btm_TL);
    		BottomGrids.add(obstacle_vector_btm_TM);
    		BottomGrids.add(obstacle_vector_btm_TR);
    		BottomGrids.add(obstacle_vector_btm_ML);
    		BottomGrids.add(obstacle_vector_btm_MM);
    		BottomGrids.add(obstacle_vector_btm_MR);
    		BottomGrids.add(obstacle_vector_btm_BL);
    		BottomGrids.add(obstacle_vector_btm_BM);
    		BottomGrids.add(obstacle_vector_btm_BR);
    		//System.out.println(BottomGrids.get(0));
    		
    		//Left of Obstacle
    		Vector2 obstacle_vector_left_TL = new Vector2(obstacle_vector.i()-3, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_left_TM = new Vector2(obstacle_vector.i()-2, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_left_TR = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_left_ML = new Vector2(obstacle_vector.i()-3, obstacle_vector.j());
    		Vector2 obstacle_vector_left_MM = new Vector2(obstacle_vector.i()-2, obstacle_vector.j());
    		Vector2 obstacle_vector_left_MR = new Vector2(obstacle_vector.i()-1, obstacle_vector.j());
    		Vector2 obstacle_vector_left_BL = new Vector2(obstacle_vector.i()-3, obstacle_vector.j()-1);
    		Vector2 obstacle_vector_left_BM = new Vector2(obstacle_vector.i()-2, obstacle_vector.j()-1);
    		Vector2 obstacle_vector_left_BR = new Vector2(obstacle_vector.i()-1, obstacle_vector.j()-1);
    		List<Vector2> LeftGrids = new ArrayList<Vector2>();
    		LeftGrids.add(obstacle_vector_left_TL);
    		LeftGrids.add(obstacle_vector_left_TM);
    		LeftGrids.add(obstacle_vector_left_TR);
    		LeftGrids.add(obstacle_vector_left_ML);
    		LeftGrids.add(obstacle_vector_left_MM);
    		LeftGrids.add(obstacle_vector_left_MR);
    		LeftGrids.add(obstacle_vector_left_BL);
    		LeftGrids.add(obstacle_vector_left_BM);
    		LeftGrids.add(obstacle_vector_left_BR);
    		//System.out.println(LeftGrids.get(0));
    		
    		
    		//Right of Obstacle
    		Vector2 obstacle_vector_right_TL = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_right_TM = new Vector2(obstacle_vector.i()+2, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_right_TR = new Vector2(obstacle_vector.i()+3, obstacle_vector.j()+1);
    		Vector2 obstacle_vector_right_ML = new Vector2(obstacle_vector.i()+1, obstacle_vector.j());
    		Vector2 obstacle_vector_right_MM = new Vector2(obstacle_vector.i()+2, obstacle_vector.j());
    		Vector2 obstacle_vector_right_MR = new Vector2(obstacle_vector.i()+3, obstacle_vector.j());
    		Vector2 obstacle_vector_right_BL = new Vector2(obstacle_vector.i()+1, obstacle_vector.j()-1);
    		Vector2 obstacle_vector_right_BM = new Vector2(obstacle_vector.i()+2, obstacle_vector.j()-1);
    		Vector2 obstacle_vector_right_BR = new Vector2(obstacle_vector.i()+3, obstacle_vector.j()-1);
    		List<Vector2> RightGrids = new ArrayList<Vector2>();
    		RightGrids.add(obstacle_vector_right_TL);
    		RightGrids.add(obstacle_vector_right_TM);
    		RightGrids.add(obstacle_vector_right_TR);
    		RightGrids.add(obstacle_vector_right_ML);
    		RightGrids.add(obstacle_vector_right_MM);
    		RightGrids.add(obstacle_vector_right_MR);
    		RightGrids.add(obstacle_vector_right_BL);
    		RightGrids.add(obstacle_vector_right_BM);
    		RightGrids.add(obstacle_vector_right_BR);
    		//System.out.println(RightGrids.get(0));
    		
    		
    		
    		
    		//test for obstacle vector state
    		//System.out.println(_wpMap[obstacle_vector.i() -1][obstacle_vector.j()].obstacleState());
    		//System.out.println(_wpMap[obstacle_vector.i() -1][obstacle_vector.j()].specialState());
    		
    		
    		/*
    		//Old first if for left 2 grids away
    		if(checkValidBoundary_ImageSensor(obstacle_vector_left_1) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_left_2) == true)
    		{
    			if(_wpMap[obstacle_vector.i() -1][obstacle_vector.j()].obstacleState() != WPObstacleState.IsActualObstacle 
    	    			&& _wpMap[obstacle_vector.i() -1][obstacle_vector.j()].specialState() != WPSpecialState.IsClosedPoint)
    	    			{
    	    				if(_wpMap[obstacle_vector.i() -2][obstacle_vector.j()].obstacleState() != WPObstacleState.IsActualObstacle
    	    						&& _wpMap[obstacle_vector.i() -2][obstacle_vector.j()].specialState() != WPSpecialState.IsClosedPoint)
    	    				{
    	    					left_list.add(obstacle_vector_left_2);
    	    					left_list.add(obstacle_vector);
    	    					listOfLists.add(left_list);
    	    					//facing True right is down
    	    		            //facing True up is right
    	    		            //facing True left is up
    	    		            //facing True down is left
    	    				}
    	    			} 
    		} // first if for left 2 grids away */
    			
    		
    		/*
    		// 2nd if for right 2 grids away
    		if(checkValidBoundary_ImageSensor(obstacle_vector_right_1) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_right_2) == true)
    		{
    			if(_wpMap[obstacle_vector.i() +1][obstacle_vector.j()].obstacleState() != WPObstacleState.IsActualObstacle 
    	    			&& _wpMap[obstacle_vector.i() +1][obstacle_vector.j()].specialState() != WPSpecialState.IsClosedPoint)
    	    			{
    	    				if(_wpMap[obstacle_vector.i() +2][obstacle_vector.j()].obstacleState() != WPObstacleState.IsActualObstacle
    	    						&& _wpMap[obstacle_vector.i() +2][obstacle_vector.j()].specialState() != WPSpecialState.IsClosedPoint)
    	    				{
    	    					right_list.add(obstacle_vector_right_2);
    	    					right_list.add(obstacle_vector);
    	    					listOfLists.add(right_list);
    	    				}
    	    			} 
    		} // 2nd if for right 2 grids away*/
    		
    		/*
    		//  Old 3rd if for top 2 grids away
    		if(checkValidBoundary_ImageSensor(obstacle_vector_top_1) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_top_2) == true)
    		{
    			if(_wpMap[obstacle_vector.i()][obstacle_vector.j()+1].obstacleState() != WPObstacleState.IsActualObstacle 
    	    			&& _wpMap[obstacle_vector.i()][obstacle_vector.j()+1].specialState() != WPSpecialState.IsClosedPoint)
    	    			{
    	    				if(_wpMap[obstacle_vector.i()][obstacle_vector.j()+2].obstacleState() != WPObstacleState.IsActualObstacle
    	    						&& _wpMap[obstacle_vector.i()][obstacle_vector.j()+2].specialState() != WPSpecialState.IsClosedPoint)
    	    				{
    	    					top_list.add(obstacle_vector_top_2);
    	    					top_list.add(obstacle_vector);
    	    					listOfLists.add(top_list);
    	    				}
    	    			} 
    		} // Old 3rd if for top 2 grids away */
    		
    		// Old 4th if for btm 2 grids away
    		/*if(checkValidBoundary_ImageSensor(obstacle_vector_btm_1) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_btm_2) == true)
    		{
    			if(_wpMap[obstacle_vector.i()][obstacle_vector.j()-1].obstacleState() != WPObstacleState.IsActualObstacle 
    	    			&& _wpMap[obstacle_vector.i()][obstacle_vector.j()-1].specialState() != WPSpecialState.IsClosedPoint)
    	    			{
    	    				if(_wpMap[obstacle_vector.i()][obstacle_vector.j()-2].obstacleState() != WPObstacleState.IsActualObstacle
    	    						&& _wpMap[obstacle_vector.i()][obstacle_vector.j()-2].specialState() != WPSpecialState.IsClosedPoint)
    	    				{
    	    					btm_list.add(obstacle_vector_btm_2);
    	    					btm_list.add(obstacle_vector);
    	    					listOfLists.add(btm_list);
    	    				}
    	    			} 
    		} //  Old 4th if for btm 2 grids away*/
    		
    		
    		
    		
    		//New Code for BTM
    		if(checkValidBoundary_ImageSensor(obstacle_vector_btm_TM) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_btm_MM) == true)
    		{
    			int checker = 0;
    			for (int item = 0; item <9; item++) 
    			{
    				if(checkValidBoundary(BottomGrids.get(item)))
    				{
    					if(_wpMap[BottomGrids.get(item).i()][BottomGrids.get(item).j()].obstacleState() != WPObstacleState.IsActualObstacle 
            	    			&& _wpMap[BottomGrids.get(item).i()][BottomGrids.get(item).j()].specialState() != WPSpecialState.IsClosedPoint)
            			{
            				checker ++;
            			}
    				}
    				
    			}
    			
    			if(checker == 9)
    			{
    				btm_list.add(obstacle_vector_btm_MM);
					btm_list.add(obstacle_vector);
					listOfLists.add(btm_list);
    			}
    			
    		}
    		
    		//New Code for LEFT
    		if(checkValidBoundary_ImageSensor(obstacle_vector_left_TM) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_left_MM) == true)
    		{
    			int checker = 0;
    			for (int item = 0; item <9; item++) 
    			{
    				if(checkValidBoundary(LeftGrids.get(item)))
    				{
    					if(_wpMap[LeftGrids.get(item).i()][LeftGrids.get(item).j()].obstacleState() != WPObstacleState.IsActualObstacle 
            	    			&& _wpMap[LeftGrids.get(item).i()][LeftGrids.get(item).j()].specialState() != WPSpecialState.IsClosedPoint)
            			{
            				checker ++;
            			}
    				}
    				
    			}
    			
    			if(checker == 9)
    			{
    				left_list.add(obstacle_vector_left_MM);
					left_list.add(obstacle_vector);
					listOfLists.add(left_list);
    			}
    			
    		}
    		
    		
    		
    		//New Code for TOP
    		if(checkValidBoundary_ImageSensor(obstacle_vector_top_TM) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_top_MM) == true)
    		{
    			int checker = 0;
    			for (int item = 0; item <9; item++) 
    			{
    				if(checkValidBoundary(TopGrids.get(item)))
    				{
    					if(_wpMap[TopGrids.get(item).i()][TopGrids.get(item).j()].obstacleState() != WPObstacleState.IsActualObstacle 
            	    			&& _wpMap[TopGrids.get(item).i()][TopGrids.get(item).j()].specialState() != WPSpecialState.IsClosedPoint)
            			{
            				checker ++;
            			}
    				}
    				
    			}
    			
    			if(checker == 9)
    			{
    				top_list.add(obstacle_vector_top_MM);
					top_list.add(obstacle_vector);
					listOfLists.add(top_list);
    			}
    			
    		}
    		
    		//New Code for RIGHT
    		if(checkValidBoundary_ImageSensor(obstacle_vector_right_TM) ==  true && checkValidBoundary_ImageSensor(obstacle_vector_right_MM) == true)
    		{
    			int checker = 0;
    			for (int item = 0; item <9; item++) 
    			{
    				if(checkValidBoundary(RightGrids.get(item)))
    				{
    					if(_wpMap[RightGrids.get(item).i()][RightGrids.get(item).j()].obstacleState() != WPObstacleState.IsActualObstacle 
            	    			&& _wpMap[RightGrids.get(item).i()][RightGrids.get(item).j()].specialState() != WPSpecialState.IsClosedPoint)
            			{
            				checker ++;
            			}
    				}
    				
    			}
    			
    			if(checker == 9)
    			{
    				right_list.add(obstacle_vector_right_MM);
					right_list.add(obstacle_vector);
					listOfLists.add(right_list);
    			}
    			
    		}
    		
   
    	}


    	return listOfLists;
    	
    }
    	
    	
    
    
    private void _processObstacleList(List<Vector2> obsList, 
                                        WPObstacleState actualState,
                                        WPObstacleState virtualCheckState,
                                        WPObstacleState virtualState) {
        obsList.forEach((curObsPos) -> {
            // add blocking tag for the obstacle
            _setObstacle(curObsPos, actualState);
            
            // add blocking tag for points adjacent to the obstacle
            for (int deltaI = -1; deltaI <= 1; deltaI++) {
                for (int deltaJ = -1; deltaJ <= 1; deltaJ++) {
                    int adjI = curObsPos.i() + deltaI;
                    int adjJ = curObsPos.j() + deltaJ;
                    if (adjI > -1 && adjI < DIM_I && adjJ > -1 && adjJ < DIM_J &&
                        _wpMap[adjI][adjJ].obstacleState() == virtualCheckState) {
                        Vector2 adjacentPos = new Vector2(adjI, adjJ);
                        _setObstacle(adjacentPos, virtualState);
                    }
                }
            }
        });
    }
    public void addObstacle(List<Vector2> obstaclePositions) {
        _processObstacleList(
            obstaclePositions,
            WPObstacleState.IsActualObstacle,
            WPObstacleState.IsWalkable,
            WPObstacleState.IsVirtualObstacle
        );
    }
    public void addObstacle(Vector2 pos) {
        List<Vector2> temp = new ArrayList<>();
        temp.add(pos);
        addObstacle(temp);
    }
    public void clearObstacle(List<Vector2> obstaclePositions) {
        _processObstacleList(
            obstaclePositions,
            WPObstacleState.IsWalkable,
            WPObstacleState.IsVirtualObstacle,
            WPObstacleState.IsWalkable
        );
    }
    public void clearObstacle(Vector2 pos) {
        List<Vector2> temp = new ArrayList<>();
        temp.add(pos);
        clearObstacle(temp);
    }
    public List<Waypoint> toList() {  //from the array of way points _wpMap, add all array elements to an arraylist and return it
        List<Waypoint> result = new ArrayList<>();
        for (int i = 0; i < DIM_I; i++) {
            for (int j = 0; j < DIM_J; j++) {
                result.add(_wpMap[i][j]);
            }
        }
        return result;
    }
    
    public void highlight(List<Vector2> hightlightPositions, WPSpecialState wPSpecialState) {
        hightlightPositions.forEach((pos) -> {
            Waypoint curPoint = _wpMap[pos.i()][pos.j()];
            if (curPoint.specialState() != WPSpecialState.IsStart &&
                curPoint.specialState() != WPSpecialState.IsGoal) {
                curPoint.specialState(wPSpecialState);
            }
        });
    }
    public void clearAllHighlight() {
        for (Waypoint[] row : _wpMap) {
            for (Waypoint wp : row) {
                if (!wp.specialState().equals(WPSpecialState.IsStart) && 
                    !wp.specialState().equals(WPSpecialState.IsGoal)) {
                    wp.specialState(WPSpecialState.NA);
                }
            }
        }
    }
    
    public String toString(Robot robot) {
        String result = "";
        for (int i = -1; i <= DIM_I; i++) {
            for (int j = -1; j <= DIM_J; j++) {
                if (i == -1 || j == -1 || i == DIM_I || j == DIM_J) {
                    result += "# ";
                } else {
                    Waypoint curPoint = _wpMap[i][j];
                    if (curPoint.position().equals(robot.position())) {
                        String symbol = "  ";
                        switch (robot.orientation()) {
                            case Up:
                                symbol = "^ ";
                                break;
                            case Down:
                                symbol = "_ ";
                                break;
                            case Left:
                                symbol = "< ";
                                break;
                            case Right:
                                symbol = "> ";
                                break;
                        }
                        result += Console.ANSI_RED + symbol + Console.ANSI_RESET;
                        continue;
                    }
                    switch (curPoint.specialState()) {
                        case IsStart:
                            result += Console.ANSI_CYAN + "S " + Console.ANSI_RESET;
                            continue;
                        case IsGoal:
                            result += Console.ANSI_PURPLE + "G " + Console.ANSI_RESET;
                            continue;
                        case IsPathPoint:
                            result += Console.ANSI_GREEN + "x " + Console.ANSI_RESET;
                            continue;
                        case IsOpenedPoint:
                            result += "x ";
                            continue;
                        case IsClosedPoint:
                            result += Console.ANSI_WHITE_BACKGROUND+ "x " + Console.ANSI_RESET;
                            continue;
                        case NA:
                            break;
                    }
                    switch (curPoint.obstacleState()) {
                        case IsActualObstacle:
                            result += Console.ANSI_WHITE_BACKGROUND + " " + Console.ANSI_RESET + " ";
                            break;
                        case IsVirtualObstacle:
                            result += "  ";
                            break;
                        case IsWalkable:
                            result += Console.ANSI_GRAY + "+ " + Console.ANSI_RESET;
                            break;
                    }
                }
            }
            result += "\n";
        }
        return result;
    }
    
    public Waypoint getPoint(Vector2 position) {
    		//System.out.println("i "+ position.i() + "j "+ position.j());
        return _wpMap[position.i()][position.j()];
    }
    
    public boolean checkValidPosition(Vector2 pos) {
        return pos.i() > 0 && pos.i() < DIM_I &&
               pos.j() > 0 && pos.j() < DIM_J;
    }
    
    public boolean checkValidBoundary(Vector2 pos){
    		return pos.i() >=0 && pos.i() < DIM_I &&
                pos.j() >=0 && pos.j() < DIM_J;
    }
    
    public boolean checkValidBoundary_ImageSensor(Vector2 pos){
		return pos.i() >=1 && pos.i() < DIM_I-1 &&
            pos.j() >=1 && pos.j() < DIM_J-1;
}
    
    
}
