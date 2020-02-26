package mdp.solver.shortestpath;

import java.util.Collections;
import java.util.HashMap;
import mdp.common.Direction;
import mdp.map.Map;
import mdp.common.Vector2;
import mdp.robot.Robot;
import mdp.map.WPObstacleState;
import mdp.map.Waypoint;

public class AStarSolver {

    private AStarSolverResult _solve(Map map, Robot robot, Vector2 goalPos, SolveType solveType) {

        AStarSolverResult result = new AStarSolverResult();

        System.out.println("Solving shortest path:");
        //System.out.println(map.toString(robot));

        // save points in map in a lookup hashtable
        HashMap<String, AStarWaypoint> openedPoints = new HashMap();
        HashMap<String, AStarWaypoint> closedPoints = new HashMap();

        // record robot position as cur point
        AStarWaypoint curPoint = new AStarWaypoint(new Waypoint(robot.position()));

        // check if cur point is goal
        if (curPoint.position().equals(goalPos)) {
            System.out.println("Robot already at goal.");
            return result;
        }

        // loop until a point next tp goal is found
        boolean isFirstCur = true;

        while (AStarUtil.getMDistance(curPoint.position(), goalPos) != 1) {
//            System.out.println("Cur:");
//            System.out.println(curPoint.position());

            // close point
            openedPoints.remove(curPoint.position().toString());
            closedPoints.put(curPoint.position().toString(), curPoint);

            // detect adj points
//            System.out.println("Finding adj:");
            Vector2 curPos = curPoint.position();
            for (Direction dir : Direction.values()) {
//                System.out.println(dir.toString());
                // get adj point
                Vector2 adjPos = curPos.fnAdd(dir.toVector2());
                if (map.checkValidPosition(adjPos)) {
                    
                    // gval
                    int baseGval;
                    int deltaGval;
                    Direction curDirection;
                    if (isFirstCur) {
                        baseGval = 0;
                        deltaGval = 1;
                    } else {
                        baseGval = curPoint.gval();
                        curDirection = curPoint.parentDir().getBehind();
                        switch (solveType) {
                            case Normal:
                            default:
                                deltaGval = AStarUtil.getMoveCost(curDirection, dir);
                                break;
                            case Smooth:
                                deltaGval = AStarUtil.getSmoothMoveCost(curDirection, dir);
                                break;

                        }
                    }
                    
                    // hval
                    int hval = AStarUtil.getMDistance(adjPos, goalPos);
                    switch (solveType) {
                        case Safe:
                            hval += AStarUtil.getSafetyBenefit(map, adjPos);
                    }
                    
                    AStarWaypoint adjPoint = new AStarWaypoint(
                            map.getPoint(adjPos),
                            hval,
                            baseGval + deltaGval,
                            dir.getBehind()
                    );

                    // check if adj is closed or already opened
                    String adjKey = adjPoint.position().toString();
                    if (!closedPoints.containsKey(adjKey)) {
                        if (openedPoints.containsKey(adjKey)) {
                            // point already in opened, update if possible
                            AStarWaypoint oldPoint = openedPoints.get(adjKey);
                            if (adjPoint.fval() < oldPoint.fval()) {
                                openedPoints.replace(adjKey, adjPoint);
                            }
                        } else {
                            // check if adj is walkable
                            if (adjPoint.obstacleState() == WPObstacleState.IsWalkable) {
                                // if yes then save new point info
                                openedPoints.put(adjKey, adjPoint);
                            }
                        }
                    }
                }
            }

            // check all open points for potential next cur point
            if (!openedPoints.isEmpty()) {
                // loop through map info to find lowest fval point
//            	System.out.println("Deciding which of the following opened points to choose:");
                AStarWaypoint lowestFvalPoint = new AStarWaypoint();
                for (String key : openedPoints.keySet()) {
                    AStarWaypoint cur = openedPoints.get(key);
//                	System.out.println(cur.position() + " hval = " + cur.hval() + ", gval = " + cur.gval() + ", fval = " + cur.fval());
                    if (cur.fval() < lowestFvalPoint.fval()) {
                        lowestFvalPoint = cur;
                    }
                }

                // set this as cur
//                System.out.println("Next cur selected:");
//                System.out.println(lowestFvalPoint.position());
//                System.out.println();
                curPoint = lowestFvalPoint;
            } else {
                // path to goal is blocked
                System.out.println("No possible path to goal found.");
                return result;
            }

//            System.out.println("Changing isFirstCur");
            if (isFirstCur) {
                isFirstCur = false;
            }
        }

//        System.out.println("Finished Finding the path...");
        // path has been found
        result.shortestPath.add(goalPos);
        do {
            // save current point to result
            result.shortestPath.add(curPoint.position());

            // trace back the parent
            Vector2 parentDirection = curPoint.parentDir().toVector2();
            Vector2 parentPos = curPoint.position().fnAdd(parentDirection);

            // set cur to parent
            curPoint = closedPoints.get(parentPos.toString());
        } while (curPoint != null && AStarUtil.getMDistance(curPoint.position(), robot.position()) != 0);
        Collections.reverse(result.shortestPath);

        // add opened & closed to result
        openedPoints.forEach((pointKey, point) -> {
            result.openedPoints.add(point.position());
        });
        closedPoints.forEach((pointKey, point) -> {
            result.closedPoints.add(point.position());
        });

        return result;

    }
    
    private AStarSolverResult _solve2(Map map, Vector2 wayPoint, Vector2 goalPos, SolveType solveType) {

        AStarSolverResult result = new AStarSolverResult();

        System.out.println("Solving shortest path:");
        //System.out.println(map.toString(robot));

        // save points in map in a lookup hashtable
        HashMap<String, AStarWaypoint> openedPoints = new HashMap();
        HashMap<String, AStarWaypoint> closedPoints = new HashMap();

        // record robot position as cur point
        AStarWaypoint curPoint = new AStarWaypoint(new Waypoint(wayPoint));

        // check if cur point is goal
        if (curPoint.position().equals(goalPos)) {
            System.out.println("Robot already at goal.");
            return result;
        }

        // loop until a point next tp goal is found
        boolean isFirstCur = true;

        while (AStarUtil.getMDistance(curPoint.position(), goalPos) != 1) {
//            System.out.println("Cur:");
//            System.out.println(curPoint.position());

            // close point
            openedPoints.remove(curPoint.position().toString());
            closedPoints.put(curPoint.position().toString(), curPoint);

            // detect adj points
//            System.out.println("Finding adj:");
            Vector2 curPos = curPoint.position();
            for (Direction dir : Direction.values()) {
//                System.out.println(dir.toString());
                // get adj point
                Vector2 adjPos = curPos.fnAdd(dir.toVector2());
                if (map.checkValidPosition(adjPos)) {
                    
                    // gval
                    int baseGval;
                    int deltaGval;
                    Direction curDirection;
                    if (isFirstCur) {
                        baseGval = 0;
                        deltaGval = 1;
                    } else {
                        baseGval = curPoint.gval();
                        curDirection = curPoint.parentDir().getBehind();
                        switch (solveType) {
                            case Normal:
                            default:
                                deltaGval = AStarUtil.getMoveCost(curDirection, dir);
                                break;
                            case Smooth:
                                deltaGval = AStarUtil.getSmoothMoveCost(curDirection, dir);
                                break;

                        }
                    }
                    
                    // hval
                    int hval = AStarUtil.getMDistance(adjPos, goalPos);
                    switch (solveType) {
                        case Safe:
                            hval += AStarUtil.getSafetyBenefit(map, adjPos);
                    }
                    
                    AStarWaypoint adjPoint = new AStarWaypoint(
                            map.getPoint(adjPos),
                            hval,
                            baseGval + deltaGval,
                            dir.getBehind()
                    );

                    // check if adj is closed or already opened
                    String adjKey = adjPoint.position().toString();
                    if (!closedPoints.containsKey(adjKey)) {
                        if (openedPoints.containsKey(adjKey)) {
                            // point already in opened, update if possible
                            AStarWaypoint oldPoint = openedPoints.get(adjKey);
                            if (adjPoint.fval() < oldPoint.fval()) {
                                openedPoints.replace(adjKey, adjPoint);
                            }
                        } else {
                            // check if adj is walkable
                            if (adjPoint.obstacleState() == WPObstacleState.IsWalkable) {
                                // if yes then save new point info
                                openedPoints.put(adjKey, adjPoint);
                            }
                        }
                    }
                }
            }

            // check all open points for potential next cur point
            if (!openedPoints.isEmpty()) {
                // loop through map info to find lowest fval point
//            	System.out.println("Deciding which of the following opened points to choose:");
                AStarWaypoint lowestFvalPoint = new AStarWaypoint();
                for (String key : openedPoints.keySet()) {
                    AStarWaypoint cur = openedPoints.get(key);
//                	System.out.println(cur.position() + " hval = " + cur.hval() + ", gval = " + cur.gval() + ", fval = " + cur.fval());
                    if (cur.fval() < lowestFvalPoint.fval()) {
                        lowestFvalPoint = cur;
                    }
                }

                // set this as cur
//                System.out.println("Next cur selected:");
//                System.out.println(lowestFvalPoint.position());
//                System.out.println();
                curPoint = lowestFvalPoint;
            } else {
                // path to goal is blocked
                System.out.println("No possible path to goal found.");
                return result;
            }

//            System.out.println("Changing isFirstCur");
            if (isFirstCur) {
                isFirstCur = false;
            }
        }

//        System.out.println("Finished Finding the path...");
        // path has been found
        result.shortestPath.add(goalPos);
        do {
            // save current point to result
            result.shortestPath.add(curPoint.position());

            // trace back the parent
            Vector2 parentDirection = curPoint.parentDir().toVector2();
            Vector2 parentPos = curPoint.position().fnAdd(parentDirection);

            // set cur to parent
            curPoint = closedPoints.get(parentPos.toString());
        } while (curPoint != null && AStarUtil.getMDistance(curPoint.position(), wayPoint) != 0);
        Collections.reverse(result.shortestPath);

        // add opened & closed to result
        openedPoints.forEach((pointKey, point) -> {
            result.openedPoints.add(point.position());
        });
        closedPoints.forEach((pointKey, point) -> {
            result.closedPoints.add(point.position());
        });

        return result;

    }

    //public AStarSolverResult solve(Map map, Robot robot, Vector2 goalPos) {
    //    return _solve(map, robot, goalPos, SolveType.Normal);
    //}

    public AStarSolverResult solve(Map map, Robot robot, Vector2 position) {
        return _solve(map, robot, position, SolveType.Normal);
    }
    
    public AStarSolverResult solve(Map map, Robot robot) {
        return _solve(map, robot, map.GOAL_POS, SolveType.Normal);
    }
    
    public AStarSolverResult solve(Map map, Robot robot, SolveType solveType) {
        return _solve(map, robot, map.GOAL_POS, solveType);
    }

    public AStarSolverResult solve(Map map, Robot robot,  Vector2 position, SolveType solveType) {
        return _solve(map, robot, position, solveType);
    }
    
    //
    public AStarSolverResult solve2(Map map, Vector2 wayPoint, Vector2 position) {
        return _solve2(map, wayPoint, position, SolveType.Normal);
    }
    
    public AStarSolverResult solve2(Map map, Vector2 wayPoint) {
        return _solve2(map, wayPoint, map.GOAL_POS, SolveType.Normal);
    }
    
    public AStarSolverResult solve2(Map map, Vector2 wayPoint, SolveType solveType) {
        return _solve2(map, wayPoint, map.GOAL_POS, solveType);
    }

    public AStarSolverResult solve2(Map map, Vector2 wayPoint,  Vector2 position, SolveType solveType) {
        return _solve2(map, wayPoint, position, solveType);
    }
//    public Vector2 solveSingle(Map map, Robot robot, Vector2 goalPos) {
//        Vector2 diff = goalPos.fnAdd(robot.position().fnMultiply(-1));
//        Vector2 horizontalResult;
//        Vector2 verticalResult;
//        if (diff.j() > 0) {
//            // move down
//            horizontalResult = robot.position().fnAdd(Direction.Down.toVector2());
//        } else {
//            // move up
//            horizontalResult = robot.position().fnAdd(Direction.Up.toVector2());
//        }
//        if (diff.i() > 0) {
//            // move right
//            verticalResult = robot.position().fnAdd(Direction.Right.toVector2());
//        } else {
//            // move left
//            verticalResult = robot.position().fnAdd(Direction.Left.toVector2());
//        }
//
//        boolean horizontallyWalkable = false;
//        boolean verticallyWalkable = false;
//        if (map.getPoint(horizontalResult).obstacleState()
//                .equals(WPObstacleState.IsWalkable)) {
//            horizontallyWalkable = true;
//        }
//        if (map.getPoint(verticalResult).obstacleState()
//                .equals(WPObstacleState.IsWalkable)) {
//            verticallyWalkable = true;
//        }
//
//        if (horizontallyWalkable) {
//            if (verticallyWalkable) {
//                return diff.i() > diff.j() ? verticalResult : horizontalResult;
//            } else {
//                return horizontalResult;
//            }
//        } else {
//            if (verticallyWalkable) {
//                return verticalResult;
//            } else {
//                System.out.println("No possible solution found.");
//                return new Vector2(-1, -1);
//            }
//        }
//    }
}
