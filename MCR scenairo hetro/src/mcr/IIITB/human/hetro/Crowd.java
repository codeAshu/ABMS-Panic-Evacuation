package mcr.IIITB.human.hetro;

import java.awt.Color;
import java.util.ArrayList;

import sim.util.*;
import sim.field.continuous.*;
import sim.field.network.Network;

import sim.engine.*;


public class Crowd  extends SimState {

	private static final long serialVersionUID = 1;

	// dimensions of the environment
	public static final double XMIN = 0;
	public static final double XMAX = 250;
	public static final double YMIN = 0;
	public static final double YMAX = 250;


	//dimension of the box
	public static final double xMinB = 20;
	public static final double xMaxB = 230;
	public static final double yMinB = 30;
	public static final double yMaxB = 150;

	//where to cut for the door in upper region
	public static final double doorLeft = 22;
	public static final double doorRight =30;
	public static final double doorMid = doorLeft + (doorRight-doorLeft)/2;



	//where to cut for the door in upper region
	public static final double lwDoorLeft =  xMaxB-10;
	public static final double lwDoorRight = xMaxB-2;
	public static final double lwDoorMid = lwDoorLeft + (lwDoorRight-lwDoorLeft)/2;


	// where the obstacles are located (diameter, xpos, ypos)
	static ArrayList< ArrayList<Double>> boundaryInfo =new ArrayList< ArrayList<Double>>();  //{ {1, 60, 40}, {15, 135, 140} };

	static ArrayList< ArrayList<Double>> obstInfo =new ArrayList< ArrayList<Double>>();

	// number of Humans
	public static final int NUM_Humans = 160;

	// the difference between simulation time and Humans time. it is used to compute by how much they moved. can be eliminated, but the speed
	// would need to be increased to maintain the same simulation quality
	public static final double TIMESTEP = 0.7;

	// for nice displaying, extra space is allocated around the visible area
	public final static double EXTRA_SPACE = -20;

	// the Humans and obstacle environments
	public static Continuous2D HumansEnvironment = null;
	public static Continuous2D obstaclesEnvironment = null;
	public static Continuous2D doorEnvironment = null;
	public static Continuous2D boxEnvironment = null;
	public static Network marking = new Network(false);

	public final Color oColor = new Color(102,178,255);

	public static int upDoorCount =0;
	public static int lwDoorCount =0;
	
	public int getLwDoorCount()
	{
		return lwDoorCount;
	}

	public int getupDoorCount()
	{
		return upDoorCount;
	}
	
	public Crowd(long seed)
	{
		super(seed);
	}

	public double[] getDistribution()
	{
		double[] dis = new double[2];
		dis[0] = (double) (lwDoorCount*100 / (lwDoorCount+upDoorCount));
		dis[1] = (double) (upDoorCount*100 / (lwDoorCount+upDoorCount));
		
		return dis;
	}
	
	public void setObjectLocation( final Human hu, Double2D location )
	{
		HumansEnvironment.setObjectLocation( hu, location );

		// to speed up the simulation, each Human knows where it is located (gets rid of a hash get call)
		hu.x = location.x;
		hu.y = location.y;
	}


	public void start()
	{
		super.start();
		HumansEnvironment = new Continuous2D( Human.MAX_DISTANCE/2, (XMAX-XMIN), (YMAX-YMIN) );
		obstaclesEnvironment = new Continuous2D( Human.MAX_DISTANCE/2, (XMAX-XMIN), (YMAX-YMIN) );
		doorEnvironment = new Continuous2D( Human.MAX_DISTANCE/2, (XMAX-XMIN), (YMAX-YMIN) );
		boxEnvironment = new Continuous2D( Human.MAX_DISTANCE/2, (XMAX-XMIN), (YMAX-YMIN) );

		//Test //add only few humans

		int lenX  = (int)(xMaxB - xMinB);
		int lenY  = (int)(yMaxB - yMinB);
		
		
		//display the boundary of the classroom
		addBoundryBox();

		//show doorArea
		int diam = (int)DoorArea.UpperVisibility*2;
		DoorArea darea1 = new DoorArea(diam);
		doorEnvironment.setObjectLocation(darea1, new Double2D(doorMid,yMinB));
		//lower door
		diam =  (int)DoorArea.lowerVisibility* 2;
		DoorArea darea2 = new DoorArea(diam);
		doorEnvironment.setObjectLocation(darea2, new Double2D(lwDoorMid,yMaxB));

		//add marking lines
		Bag bag = boxEnvironment.getNeighborsExactlyWithinDistance(new Double2D(xMaxB,yMinB),5 );
		Obstacle node1 = (Obstacle) bag.get(0);
		bag = boxEnvironment.getNeighborsExactlyWithinDistance(new Double2D(xMaxB-40,yMaxB),5 );
		Obstacle node2 = (Obstacle) bag.get(0);
		marking.addEdge(node1, node2, null);

		bag = boxEnvironment.getNeighborsExactlyWithinDistance(new Double2D(xMinB,yMinB),5 );
		node1 = (Obstacle) bag.get(0);
		bag = boxEnvironment.getNeighborsExactlyWithinDistance(new Double2D(xMinB+40,yMaxB),5 );
		node2 = (Obstacle) bag.get(0);
		marking.addEdge(node1, node2, null);


		// add the seats as obstacles and humans above them to the simulation
		addSeatsAndHumans();

	}

	/***********************************
	 * Create the box for the simulation
	 * 
	 ***********************************/
	public void addSeatsAndHumans()
	{
		//left most section tilted
		double p1  = 60;
		double p2 = 132;
		double oDiameter = 5;
		
		//first row before the seats adding humans
		double h1 = p1+2.5;
		double h2 = p2+7;
		for (int i = 0; i <4; i++){
			Human hu = null;
			double diameter = 2 + random.nextInt(1) + random.nextFloat();	
			double mass = diameter*10 + random.nextInt(20);
			hu = new Human(diameter,mass);
			hu.x = h1;
			hu.y = h2;
			schedule.scheduleRepeating(hu);
			HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );
			h1+=4;h2-=4;
		}
		//rest of the rows
		for (int j = 0; j <8 ; j++) 
		{
			double n1  = p1;
			double n2 = p2;
			p1=p1-3.5;
			p2=p2-12;

			//left most section tilted
			for (int i = 0; i <4; i++) {
				ArrayList<Double> arg = new ArrayList<Double>();

				Obstacle ob = new Obstacle(oDiameter,oColor);
				obstaclesEnvironment.setObjectLocation(ob, new Double2D(n1,n2));
				arg.add(oDiameter); arg.add(n1); arg.add(n2);
				obstInfo.add(arg);														//obstacles-seats

				Human hu = null;
				double diameter = 2 + random.nextInt(1) + random.nextFloat();	
				double mass = diameter*10 + random.nextInt(20);
				hu = new Human(diameter,mass);
				h1 = n1+0.25;
				h2 = n2-8;
				hu.x = h1;
				hu.y = h2;
				schedule.scheduleRepeating(hu);
				if(j<7)
					HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );	//humans

				n1+=4;n2-=4;
			}
		}
	
		
		//middle section
		p1  = 90;
		p2 = 116;

		//first row of humans
		h1 = p1;
		h2 = p2+6;
		for (int i = 0; i <6; i++){
			Human hu = null;
			double diameter = 2 + random.nextInt(1) + random.nextFloat();	
			double mass = diameter*10 + random.nextInt(20);
			hu = new Human(diameter,mass);
			hu.x = h1;
			hu.y = h2;
			schedule.scheduleRepeating(hu);
			HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );
			h1+=6;
		}
		//rest of the rows
		for (int j = 0; j <8 ; j++) {

			double n1  = p1;
			double n2  = p2;

			p2=p2-12;

			for (int i = 0; i <6; i++) {
				ArrayList<Double> arg = new ArrayList<Double>();

				Obstacle ob = new Obstacle(oDiameter,oColor);
				obstaclesEnvironment.setObjectLocation(ob, new Double2D(n1,n2));

				arg.add(oDiameter); arg.add(n1); arg.add(n2);
				obstInfo.add(arg);

				Human hu = null;
				double diameter = 2 + random.nextInt(1) + random.nextFloat();	
				double mass = diameter*10 + random.nextInt(20);
				hu = new Human(diameter,mass);
				h1 = n1;
				h2 = n2-6;
				hu.x = h1;
				hu.y = h2;
				schedule.scheduleRepeating(hu);
				if(j<7)
					HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );

				n1+=6;
			}
		}


		//middle section
		p1  = 140;
		p2 = 116;

		h1 = p1;
		h2 = p2+6;
		for (int i = 0; i <6; i++){
			Human hu = null;
			double diameter = 2.5 + random.nextInt(1) + random.nextFloat();	
			double mass = diameter*10 + random.nextInt(20);
			hu = new Human(diameter,mass);
			hu.x = h1;
			hu.y = h2;
			schedule.scheduleRepeating(hu);
			HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );
			h1+=6;
		}

		for (int j = 0; j <8 ; j++) {

			double n1  = p1;
			double n2  = p2;

			p2=p2-12;

			for (int i = 0; i <6; i++) {
				ArrayList<Double> arg = new ArrayList<Double>();

				Obstacle ob = new Obstacle(oDiameter,oColor);
				obstaclesEnvironment.setObjectLocation(ob, new Double2D(n1,n2));

				arg.add(oDiameter); arg.add(n1); arg.add(n2);
				obstInfo.add(arg);

				Human hu = null;
				double diameter = 2 + random.nextInt(1) + random.nextFloat();	
				double mass = diameter*10 + random.nextInt(20);
				hu = new Human(diameter,mass);
				h1 = n1;
				h2 = n2-6;
				hu.x = h1;
				hu.y = h2;
				schedule.scheduleRepeating(hu);
				if(j<7)
					HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );

				n1+=6;
			}
		}

		//rightmost section
		p1  = 182;
		p2 = 120;
		
		 h1 = p1-2.5;
		 h2 = p2+7;
			for (int i = 0; i <4; i++){
				Human hu = null;
				double diameter = 2 + random.nextInt(1) + random.nextFloat();	
				double mass = diameter*10 + random.nextInt(20);
				hu = new Human(diameter,mass);
				hu.x = h1;
				hu.y = h2;
				schedule.scheduleRepeating(hu);
				HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );
				h1+=4;h2+=4;
			}
		for (int j = 0; j <8 ; j++) {

			double n1  = p1;
			double n2 = p2;

			for (int i = 0; i <4; i++) {
				ArrayList<Double> arg = new ArrayList<Double>();

				Obstacle ob = new Obstacle(oDiameter,oColor);
				obstaclesEnvironment.setObjectLocation(ob, new Double2D(n1,n2));

				arg.add(oDiameter); arg.add(n1); arg.add(n2);
				obstInfo.add(arg);

				Human hu = null;
				double diameter = 2 + random.nextInt(1) + random.nextFloat();	
				double mass = diameter*10 + random.nextInt(20);
				hu = new Human(diameter,mass);
				h1 = n1-0.25;
				h2 = n2-8;
				hu.x = h1;
				hu.y = h2;
				schedule.scheduleRepeating(hu);
				if(j<7)
					HumansEnvironment.setObjectLocation( hu,  new Double2D(h1,h2) );
				
				n1+=4;n2+=4;

			}
			p1=p1+3;
			p2=p2-12;
		}
	}

	/***********************************
	 * Create the box for the simulation
	 * 
	 ***********************************/
	public void addBoundryBox(){
		Color c = Color.red;

		//upper wall vary x y is min fixed
		for(double i = xMinB ; i <xMaxB ; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(i);arg.add(yMinB);

			if(i<doorLeft || i>doorRight)			//door in the room
				boundaryInfo.add(arg);
		}

		//lower wall vary x y is max fixed
		for(double i = xMinB ; i <xMaxB ; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(i);arg.add(yMaxB);

			if(i<lwDoorLeft || i>lwDoorRight)			//door in the room
				boundaryInfo.add(arg);
		}

		//left wall x is min and y will vary

		for(double i = yMinB ; i <yMaxB; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(xMinB);arg.add(i);
			boundaryInfo.add(arg);
		}

		//right wall, x is max and y vary
		for(double i = yMinB ; i <yMaxB; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(xMaxB);arg.add(i);
			boundaryInfo.add(arg);
		}
		//add to the environment
		for (int i = 0; i < boundaryInfo.size(); i++) {

			Obstacle obst = new Obstacle( boundaryInfo.get(i).get(0),c);
			boxEnvironment.setObjectLocation( obst, new Double2D( boundaryInfo.get(i).get(1), boundaryInfo.get(i).get(2)) );
			marking.addNode(obst);
			
		}

	}
	
	//model inspector
	//APR13
	public int getCount()
	{
		return HumansEnvironment.getAllObjects().numObjs;
	}
	//APR13
	public static void main(String[] args)
	{
		doLoop(Crowd.class, args);
		System.exit(0);
	}    
}


