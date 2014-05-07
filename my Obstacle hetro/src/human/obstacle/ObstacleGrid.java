
package human.obstacle;

import java.awt.*;

import sim.portrayal.simple.*;

public class ObstacleGrid extends ShapePortrayal2D
{
	

	public ObstacleGrid(double[] xpoints, double[] ypoints, Paint paint,
			double scale, boolean filled) {
		
		super(xpoints, ypoints, paint, scale, filled);
		setStroke(new BasicStroke(12) ); 
			
			
	}

	private static final long serialVersionUID = 1;

	public final static Paint obstacleColor = new Color(192,255,192);
	// gradient obstacles!  Try it!  Slower but fun!
	// public final static Paint obstacleColor = new GradientPaint(0,0,Color.red,10,10,Color.green,true);
	
	
	
	
/*	
	public void draw(Object object,  final Graphics2D g, final DrawInfo2D info )
	{
		g.setStroke(new BasicStroke(12));
		paint = Color.black;
		
		
		info.draw.width  = 160;
		info.draw.height  = 120;
		filled = false;
		scale = 3.2;
		super.draw(object, g, info);
		
		int doorWidth =  (Crowd.doorRight - Crowd.doorLeft);
		int doorCenter = Crowd.doorLeft + doorWidth/2;
		g.setColor(CrowdWithUI.c);
		
		g.clearRect(Crowd.doorLeft*3, (int) Crowd.yMinB*3, doorWidth*4, 20);
	*/
	
	
	}

	

