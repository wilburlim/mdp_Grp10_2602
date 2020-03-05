package mdp.robot;

import java.util.LinkedList;
import java.util.List;
import mdp.common.Direction;
import mdp.common.Vector2;

public enum RobotAction {
    MoveForward, MoveBackward, RotateLeft, RotateRight;
    
    public static LinkedList<RobotAction> fromPath(Robot robot, List<Vector2> path) {
        LinkedList<RobotAction> result = new LinkedList<>();
        
        Vector2 curPos = robot.position();
        Robot tempRobot = new Robot(
            new Vector2(robot.position().i(), robot.position().j()),
            robot.orientation()
        );
        for (Vector2 curPathPos : path) {
            Vector2 diff = curPathPos.fnAdd(curPos.fnMultiply(-1));
            for (Direction dir : Direction.values()) {
                // check next pos direction
                if (diff.equals(dir.toVector2())) {
                    // check orientation
                    if (diff.equals(tempRobot.orientation().getLeft().toVector2())) {
                        tempRobot.execute(RotateLeft);
                        result.add(RotateLeft);
                    } else if (diff.equals(tempRobot.orientation().getRight().toVector2())) {
                        tempRobot.execute(RotateRight);
                        result.add(RotateRight);
                    } else if (diff.equals(tempRobot.orientation().getBehind().toVector2())) {
                        tempRobot.execute(RotateLeft);
                        tempRobot.execute(RotateLeft);
                        result.add(RotateLeft);
                        result.add(RotateLeft);
                    }
                    tempRobot.execute(MoveForward);
                    result.add(MoveForward);
                }
            }
            curPos = curPathPos;
        }
        
        return result;        
    }
    
    //New item
    
    public static ImagePath fromPath_SetPos(Robot robot, List<Vector2> path, Vector2 PrevWP, Direction PrevDir) {
        LinkedList<RobotAction> result = new LinkedList<>();
        
        
        Vector2 curPos = PrevWP;
        Robot tempRobot = new Robot(
            new Vector2(PrevWP.i(), PrevWP.j()),
            //facing right is down
            //facing up is right
            //facing left is up
            //facing down is left
            PrevDir // Pls change this var	
            //robot.orientation()
        );
        for (Vector2 curPathPos : path) {
            Vector2 diff = curPathPos.fnAdd(curPos.fnMultiply(-1));
            for (Direction dir : Direction.values()) {
                // check next pos direction
                if (diff.equals(dir.toVector2())) {
                    // check orientation
                    if (diff.equals(tempRobot.orientation().getLeft().toVector2())) {
                        tempRobot.execute(RotateLeft);
                        result.add(RotateLeft);
                    } else if (diff.equals(tempRobot.orientation().getRight().toVector2())) {
                        tempRobot.execute(RotateRight);
                        result.add(RotateRight);
                    } else if (diff.equals(tempRobot.orientation().getBehind().toVector2())) {
                        tempRobot.execute(RotateLeft);
                        tempRobot.execute(RotateLeft);
                        result.add(RotateLeft);
                        result.add(RotateLeft);
                    }
                    tempRobot.execute(MoveForward);
                    result.add(MoveForward);
                }
            }
            curPos = curPathPos;
        }
        
        System.out.println("Current Position 2 grids from obs = " + tempRobot.position());
        System.out.println("Current Orientation 2 grids from obs = " + tempRobot.orientation() + "\n");
        
        
        Direction current_orientation = tempRobot.orientation();
        
        ImagePath resultRobotImage = new ImagePath(result, current_orientation);
        
        return resultRobotImage;   
    }
}