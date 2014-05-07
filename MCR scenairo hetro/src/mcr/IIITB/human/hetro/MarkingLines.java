package mcr.IIITB.human.hetro;

import java.awt.Color;
import java.awt.Graphics2D;

import sim.portrayal.DrawInfo2D;
import sim.portrayal.SimplePortrayal2D;
import sim.portrayal.simple.OrientedPortrayal2D;
import sim.util.Double2D;

public class MarkingLines extends OrientedPortrayal2D{
	
	private static final long serialVersionUID = 1;
	Double2D p1 = new Double2D(0,0);
	Double2D p2 = new Double2D(0,0);

	public MarkingLines(Double2D p1, Double2D p2)
	{
		
		super(null, 0, 10, Color.black);
		this.p1 = p1;
		this.p2 = p2;
		
	}
	
	 
	
}
