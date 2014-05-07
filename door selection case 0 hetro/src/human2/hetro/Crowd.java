package human2.hetro;

import java.util.ArrayList;

import sim.util.*;
import sim.field.continuous.*;

import sim.engine.*;


public class Crowd  extends SimState {

	private static final long serialVersionUID = 1;

	// dimensions of the environment
	public static final double XMIN = 0;
	public static final double XMAX = 190;
	public static final double YMIN = 0;
	public static final double YMAX = 190;


	//dimension of the box
	public static final double xMinB = 20;
	public static final double xMaxB = 180;
	public static final double yMinB = 60;
	public static final double yMaxB = 180;

	//where to cut for the door
	public static final double doorLeft = 104;
	public static final double doorRight =110;
	public static final double doorMid =(doorLeft+doorRight)/2;

	// where the obstacles are located (diameter, xpos, ypos)
	static ArrayList< ArrayList<Double>> obstInfo =new ArrayList< ArrayList<Double>>();  //{ {1, 60, 40}, {15, 135, 140} };


	// number of Humans
	public static final int NUM_Humans = 400;

	// the difference between simulation time and Humans time. it is used to compute by how much they moved. can be eliminated, but the speed
	// would need to be increased to maintain the same simulation quality
	public static final double TIMESTEP =  1;

	// for nice displaying, extra space is allocated around the visible area
	public final static double EXTRA_SPACE = -20;

	// the Humans and obstacle environments
	public static Continuous2D HumansEnvironment = null;
	public Continuous2D obstaclesEnvironment = null;



	public Crowd(long seed)
	{
		super(seed);
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
		HumansEnvironment = new Continuous2D( Human.MAX_DISTANCE/3, (XMAX-XMIN), (YMAX-YMIN) );
		obstaclesEnvironment = new Continuous2D( /*Math.max(hu.numLinks*DIAMETER,30)*/130, (XMAX-XMIN), (YMAX-YMIN) );


		//Test //add only few humans

		int lenX  = (int)(xMaxB - xMinB);
		int lenY  = (int)(yMaxB - yMinB);

		for(int x=0;x<NUM_Humans;x++)
		{
			Double2D loc = null;
			Human hu = null;
			double diameter = 3 + random.nextInt(1) + random.nextFloat();	
			double mass = diameter*10 + random.nextInt(20);
			hu = new Human(diameter,mass);
			loc = new Double2D(xMinB + random.nextInt(lenX-10) + 4.5, yMinB+ random.nextInt(lenY-10) + 4.5);
			HumansEnvironment.setObjectLocation( hu, loc );
			hu.x = loc.x;
			hu.y = loc.y;
			schedule.scheduleRepeating(hu);
			//p+=random.nextInt(15); j+=random.nextInt(25);

			//p+=random.nextInt(15); j +=random.nextInt(25);

		}

		// add the obstacles to the simulation
		addObstacle();

		/*
		for( int i = 0 ; i < obstInfo.length ; i++ )
            {
            Obstacle obst = new Obstacle( obstInfo[i][0] );
            obstaclesEnvironment.setObjectLocation( obst, new Double2D( obstInfo[i][1], obstInfo[i][2] ) );


           }
		 */
		//ObstacleGrid ob = new ObstacleGrid();
		//obstaclesEnvironment.setObjectLocation(ob, new Double2D(100,100));

	}

	/***********************************
	 * Create the box of obstacles
	 * 
	 ***********************************/
	public void addObstacle() {

		int length = 180;

		//upper wall vary x y is min fixed
		for(double i = xMinB ; i <length ; i+=4 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(i);arg.add(yMinB);



			if(i<doorLeft || i>doorRight)			//door in the room
				obstInfo.add(arg);
		}

		for (int i = 0; i < obstInfo.size(); i++) {

			Obstacle obst = new Obstacle( obstInfo.get(i).get(0) );
			obstaclesEnvironment.setObjectLocation( obst, new Double2D( obstInfo.get(i).get(1), obstInfo.get(i).get(2)) );
		}
		obstInfo.clear();


		//lower wall vary x y is max fixed
		for(double i = xMinB ; i <length ; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(i);arg.add(yMaxB);
			obstInfo.add(arg);
		}

		for (int i = 0; i < obstInfo.size(); i++) {

			Obstacle obst = new Obstacle( obstInfo.get(i).get(0) );
			obstaclesEnvironment.setObjectLocation( obst, new Double2D( obstInfo.get(i).get(1), obstInfo.get(i).get(2)) );
		}

		//left wall x is min and y will vary
		obstInfo.clear();
		for(double i = yMinB ; i <length; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(xMinB);arg.add(i);
			obstInfo.add(arg);
		}
		for (int i = 0; i < obstInfo.size(); i++) {

			Obstacle obst = new Obstacle( obstInfo.get(i).get(0) );
			obstaclesEnvironment.setObjectLocation( obst, new Double2D( obstInfo.get(i).get(1), obstInfo.get(i).get(2)) );
		}

		//right wall, x is max and y vary
		for(double i = yMinB ; i <length; i+=3 )
		{

			ArrayList<Double> arg = new ArrayList<Double>();
			arg.add(2.0);arg.add(xMaxB);arg.add(i);
			obstInfo.add(arg);
		}
		for (int i = 0; i < obstInfo.size(); i++) {

			Obstacle obst = new Obstacle( obstInfo.get(i).get(0) );
			obstaclesEnvironment.setObjectLocation( obst, new Double2D( obstInfo.get(i).get(1), obstInfo.get(i).get(2)) );
		}

	}


	public int getCount()
	{
		return (400-HumansEnvironment.getAllObjects().numObjs);
	}


	public Double[] getLeftCountThreeToFour()
	{
		Bag allHumans = new Bag();
		allHumans = HumansEnvironment.getAllObjects() ;
		ArrayList<Double> allRadius = new ArrayList<Double>();

		for (int i = 0; i < allHumans.numObjs; i++) {
			Human p = (Human)(allHumans.objs[i]);

			if(p.getRadius()<4 && p.getRadius()>3)
				allRadius.add(p.getRadius());

		}
		Double[] radius = new Double[allRadius.size()];
		int i =0;
		for (Double double1 : allRadius) {
			radius[i] = double1;
		}

		return  radius ;

	}


	public static void main(String[] args)
	{
		doLoop(Crowd.class, args);
		System.exit(0);
	}    
}


