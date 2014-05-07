
package human.obstacle;

import java.awt.*;
import java.util.ArrayList;

import com.vividsolutions.jts.geomgraph.Position;

import sim.portrayal.simple.*;
import sim.util.Double2D;

public class Obstacle extends OvalPortrayal2D
    {
    private static final long serialVersionUID = 1;

    public final static Paint obstacleColor = new Color(192,255,192);
    // gradient obstacles!  Try it!  Slower but fun!
    // public final static Paint obstacleColor = new GradientPaint(0,0,Color.red,10,10,Color.green,true);
    
    public int diameter = 0;
    
    public Obstacle(double diam,Color c)
        {
    
        super(c,diam);
        this.diameter = (int)diam;
        }

	public ArrayList<Position> getVertices() {
		// TODO Auto-generated method stub
		return null;
	}

	public int getDiammeter() {
		
		return diameter;
	}
	
	public Double2D getlocation()
	{
		
		Double2D oLoc = Crowd.obstaclesEnvironment.getObjectLocationAsDouble2D(this);
		return oLoc;
		
	}
    }
