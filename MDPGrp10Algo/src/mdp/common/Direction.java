
package mdp.common;


public enum Direction {
    Up, Left, Down, Right;
    private Direction _getWithOffset(int offset) {
        return values()[(this.ordinal() + offset + values().length) % values().length];
    }
    public Direction getLeft() {
        return _getWithOffset(1);
    }
    public Direction getRight() {
        return _getWithOffset(-1);
    }
    public Direction getBehind() {
        return _getWithOffset(2);
    }
    public Vector2 toVector2() {
        switch (this) {
            case Up: return new Vector2(-1, 0);
            case Down: return new Vector2(1, 0);
            case Left: return new Vector2(0, -1);
            case Right: return new Vector2(0, 1);
            default: return new Vector2(0, 0);
        }
    }
    
    public static Direction getDesiredOrientation(Vector2 ObstacleWayPoint, Vector2 ObstaclePosition)
    {
    	int x = ObstaclePosition.i()-ObstacleWayPoint.i();
    	int y = ObstaclePosition.j()-ObstacleWayPoint.j();
    	Direction result = Direction.Right;
    	 //facing right is down
        //facing up is right
        //facing left is up
        //facing down is left
    	if(x > 0 && y == 0)
    	{
    		result = Direction.Down;
    	}
    	if(x < 0 && y == 0)
    	{
    		result =  Direction.Up;
    	}
    	if(x == 0 && y > 0)
    	{
    		result =  Direction.Right;
    	}
    	if(x == 0 && y < 0)
    	{
    		result =  Direction.Left;
    	}
    	return result;
    		  	
    }
}