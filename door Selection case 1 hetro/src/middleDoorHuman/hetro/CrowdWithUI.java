package middleDoorHuman.hetro;

import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.continuous.*;
import sim.portrayal.simple.CircledPortrayal2D;
import sim.portrayal.simple.LabelledPortrayal2D;
import sim.portrayal.simple.MovablePortrayal2D;
import sim.portrayal.simple.OvalPortrayal2D;
import sim.engine.*;
import sim.display.*;

import javax.swing.*;
import java.awt.Color;
import java.awt.Graphics2D;
public class CrowdWithUI extends GUIState{

	public Display2D display;
	public JFrame displayFrame;
	Color c =   new Color(214,255,255);

	ContinuousPortrayal2D humansPortrayal = new ContinuousPortrayal2D();
	ContinuousPortrayal2D obstaclesPortrayal = new ContinuousPortrayal2D();
	ContinuousPortrayal2D doorPortrayal = new ContinuousPortrayal2D();

	public static void main(String[] args)
	{
		new CrowdWithUI().createController();


	}
	public CrowdWithUI() { super(new Crowd(System.currentTimeMillis())); }

	public CrowdWithUI(SimState state) 
	{
		super(state);

	}

	public static String getName()
	{
		return "Middle door Hetrogenious ";
	}

	public void start()
	{
		super.start();
		setupPortrayals();
	}

	public void load(SimState state)
	{
		super.load(state);
		setupPortrayals();
	}


	public void setupPortrayals()
	{
		// tell the portrays what to portray and how to portray them
				Crowd crowd = (Crowd) state;
				humansPortrayal.setField(Crowd.HumansEnvironment);
				humansPortrayal.setPortrayalForAll(new MovablePortrayal2D(
						new CircledPortrayal2D(
								new LabelledPortrayal2D(
										new OvalPortrayal2D()
										{
											public void draw(Object object, Graphics2D g , DrawInfo2D info)
											{
												Human human = (Human)object;

												int agitationShade = (int) (human.getPanic() * 255*3);
												if (agitationShade > 255) agitationShade = 255;
												paint = new Color(  agitationShade,  255 - agitationShade,120);
												filled = true;
												//	paint = Color.black;
												scale = human.diameter;

												//draw circle
												super.draw(human, g, info);
												if(human.dead == true)
													{
													filled = true;
													paint = Color.DARK_GRAY;
													}
												else
													{
													filled =false;
													paint = Color.black;
													}

												
												super.draw(human, g, info);

												// draw our line as well
												final double width = info.draw.width * human.diameter ;
												final double height = info.draw.height * human.diameter;

												g.setColor(Color.BLACK);
												double d = human.dvel.angle();
												g.drawLine((int)info.draw.x,
														(int)info.draw.y,
														(int)(info.draw.x) + (int)(width/2 * /*Strict*/Math.cos(d)),
														(int)(info.draw.y) + (int)(height/2 * /*Strict*/Math.sin(d)));
											}
										}, 
										7.0, null, Color.black, true),
										0, 7.0, Color.black, true)));

		doorPortrayal.setField(Crowd.doorEnvironment);
		obstaclesPortrayal.setField(((Crowd)state).obstaclesEnvironment);

		// reschedule the displayer
		display.reset();
		display.setBackdrop(c);

		// redraw the display
		display.repaint();
	}

	public void init(Controller c)
	{
		super.init(c);

		// make the displayer
		display = new Display2D(600,600,this);


		displayFrame = display.createFrame();
		displayFrame.setTitle("Humans Demonstration Display");
		c.registerFrame(displayFrame);   // register the frame so it appears in the "Display" list
		displayFrame.setVisible(true);
		display.attach( doorPortrayal, "Door area" );
		display.attach( humansPortrayal, "humans" );
		display.attach( obstaclesPortrayal, "Obstacles" );
	}


	public Object getSimulationInspectedObject() { return state; }
	public Inspector getInspector()
	{
		Inspector i = super.getInspector();
		i.setVolatile(true);
		return i;
	}
	public void quit()
	{
		super.quit();
		if (displayFrame!=null) displayFrame.dispose();
		displayFrame = null;
		display = null;
	}
}
