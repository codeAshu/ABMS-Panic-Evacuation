
package middleDoorHuman.hetro;

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

	public DoorArea(int diameter)
	{
		
		super(doorColor,diameter,false);

	}

	
	
}
