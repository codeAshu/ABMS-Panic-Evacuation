
package cornorDoorHuman.hetro;

import java.awt.*;
import java.util.ArrayList;

 

import com.vividsolutions.jts.geomgraph.Position;

import sim.portrayal.simple.*;

public class DoorArea extends OvalPortrayal2D
{
	private static final long serialVersionUID = 1;

//	public final static Paint obstacleColor = new Color(192,255,192);
	// gradient obstacles!  Try it!  Slower but fun!
	 public final static Paint doorColor = new GradientPaint(0,0,Color.red,10,10,Color.green,true);
	
	 private static final int DIAMETER = 4;
	 public static double UpperVisibility =(int)(Crowd.yMaxB-Crowd.yMinB);  //visibility of the upper door  
	 public static double lowerVisibility=(int)(Crowd.yMaxB-Crowd.yMinB);   //visibility of the  lower door


	public DoorArea(int diameter)
	{
		
		super(doorColor,diameter,false);

	}

	public ArrayList<Position> getVertices() {
		// TODO Auto-generated method stub
		return null;
	}
}
