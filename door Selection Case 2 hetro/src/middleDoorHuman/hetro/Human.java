package middleDoorHuman.hetro;

import sim.engine.SimState;
import sim.engine.Steppable;
import java.util.ArrayList;
import javax.vecmath.Tuple2d;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

//done the work!!
public class Human implements Steppable {

	public static final double MAX_FORCE = 4.0;
	private static final long serialVersionUID = 1;
	double diameter ;
	private double mass;
	public static int trigger = 0;


	public static final double CENTROID_DISTANCE = 20 ;
	public  double avoiDistance = 10;
	public static final double COPY_SPEED_DISTANCE = 10;

	public static  double ADJUSTMENT_RATE = 0.05;
	public static double MAX_DISTANCE = /*Strict*/Math.max( CENTROID_DISTANCE,
			/*Strict*/Math.max( 6,
					COPY_SPEED_DISTANCE ) );

	 
	public static final double MIN_VELOCITY = 0.35;
	public static final double MAX_VELOCITY = 0.95;

	public static double WALL_MULT = 0.4;		//repulsion from wall and other immobile obstacle.

	public double panic =0.05 ;	//Trigger PANIC
	//if Panic it will be very less 0.02 else 0.5
	public static double PANIC_MULT = 0.8;		//repulsion from wall and other immobile obstacle.

	//door visibilities
	public  double UpperVisibility ;//= (Crowd.yMaxB - Crowd.yMinB)/2;  //visibility of the door  //visibility of the door
	public  double lowerVisibility;// =
	private double deathForce;
	boolean dead = false;
	
	public Human(double diameter, double mass) {
		this.diameter =diameter;
		this.mass = mass;

	}

	public final double distanceSquared( final Vector2D loc1, final Vector2D loc2 )
	{
		return( (loc1.x-loc2.x)*(loc1.x-loc2.x)+(loc1.y-loc2.y)*(loc1.y-loc2.y) );
	}

	// squared distance between two points
	public final double distanceSquared( final Vector2D loc1, final Double2D loc2 )
	{
		return( (loc1.x-loc2.x)*(loc1.x-loc2.x)+(loc1.y-loc2.y)*(loc1.y-loc2.y) );
	}

	// squared distance between two points
	public final double distanceSquared( final double x1, final double y1, final double x2, final double y2 )
	{
		return ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	}

	// dot product between two vectors
	public final double dotproduct( final Vector2D loc1, final Vector2D loc2 )
	{
		return loc1.x*loc2.x+loc1.y*loc2.y;
	}


	//initializes distances to close by Humans. it should be called a single time in the step function at each timestep.
	Bag nearbyHuman;
	double[] distSqrTo;
	public double x;
	public double y;
	Bag hum;
	Bag obs;

	/**************************************************************
	 * get nearBy humans and position of each human
	 * @param state
	 * @param pos
	 * @param distance
	 *************************************************************/
	void preprocessHumans( final Crowd state, Double2D pos, double distance )
	{
		nearbyHuman = Crowd.HumansEnvironment.getNeighborsWithinDistance( pos, distance );
		if( nearbyHuman == null )
		{
			return;
		}
		distSqrTo = new double[nearbyHuman.numObjs];
		for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
		{
			Human p = (Human)(nearbyHuman.objs[i]);
			distSqrTo[i] = distanceSquared(
					pos.x,pos.y,p.x,p.y);
		}
	}


	protected Vector2D HumanPosition = new Vector2D( 0, 0 );

	/*************************************************
	 * get flock's center of mass :Cohesion
	 * returns a vector towards the center of the flock 
	 ****************************************************/
	public Vector2D towardsFlockCenterOfMass( final Crowd state )
	{
		if( nearbyHuman == null )
			return new Vector2D( 0, 0 );
		Vector2D mean = new Vector2D( 0, 0 );
		int n = 0;
		for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
		{
			if( nearbyHuman.objs[i] != this &&
					distSqrTo[i] <= CENTROID_DISTANCE * CENTROID_DISTANCE &&
					distSqrTo[i] > this.avoiDistance * this.avoiDistance )
			{
				Human p = (Human)(nearbyHuman.objs[i]);
				mean = mean.add(new Double2D(p.x,p.y));
				n++;
			}
		}
		if( n == 0 )
			return new Vector2D( 0, 0 );
		else
		{
			mean = mean.amplify( 1.0 / n );
			mean = mean.subtract( HumanPosition );
			return mean.normalize();
		}
	}


	/******************************************************
	 *  returns a vector away from humans that are too close
	 * @param state
	 * @return
	 * Separations or avoid 
	 ********************************************************/
	public Vector2D awayFromCloseBys( final Crowd state )
	{
		if( nearbyHuman == null )
			return new Vector2D( 0, 0 );
		Vector2D away = new Vector2D( 0, 0 );
		for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
		{
			if( nearbyHuman.objs[i] != this &&
					distSqrTo[i] <= this.avoiDistance * this.avoiDistance )
			{
				Human p = (Human)(nearbyHuman.objs[i]);
				Vector2D temp = HumanPosition.subtract(new Double2D(p.x,p.y));
				temp = temp.normalize();
				away = away.add( temp ); 

			}
		}
		return away.normalize();
	}


	protected Vector2D velocity = new Vector2D( 0, 0 );
	MutableDouble2D dvel = new MutableDouble2D(0, 0);
	protected Vector2D acceleration = new Vector2D( 0, 0 );
	/***************************************************
	 * returns the mean speed of the nearby Humans
	 * @param state
	 * @return
	 * Alignments
	 ******************************************************/
	public Vector2D matchFlockSpeed( final SimState state )
	{
		if( nearbyHuman == null )
			return new Vector2D( 0, 0 );
		Vector2D mean = new Vector2D( 0, 0 );
		int n = 0;
		for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
		{
			if( nearbyHuman.objs[i] != this &&
					distSqrTo[i] <= COPY_SPEED_DISTANCE * COPY_SPEED_DISTANCE &&
					distSqrTo[i] > this.avoiDistance * this.avoiDistance )
			{
				mean = mean.add( ((Human)(nearbyHuman.objs[i])).velocity );
				n++;
			}
		}
		if( n == 0 )
			return new Vector2D( 0, 0 );
		else
		{
			mean = mean.amplify( 1.0 / n );
			return mean.normalize();
		}
	}

	/****************************************************************
	 *If alone or very slow get a random direction to go
	 *
	 ********************************************************************/
	//returns a random directions
	public Vector2D randomDirection( final Crowd state )
	{
		if(nearbyHuman == null || velocity.length()<MIN_VELOCITY)
		{
			Vector2D temp = new Vector2D( 1.0 - 2.0 * state.random.nextDouble(),
					1.0 - 2.0 * state.random.nextDouble() ).amplify(5);
			return temp.setLength( MIN_VELOCITY + state.random.nextDouble()*(MAX_VELOCITY-MIN_VELOCITY) );
		}
		else
			return new Vector2D(0,0);
	}


	/****************************************************
	 * obstacle avoidance algo.returns a direction away from obstacles
	 * @param state
	 * @return
	 ***********************************************************/
	public Vector2D avoidObstacles( final SimState state )
	{
		ArrayList< ArrayList<Double>> info =  Crowd.obstInfo;

		if( info == null || info.size() == 0 )
			return new Vector2D( 0, 0 );

		Vector2D away = new Vector2D( 0, 0 );
		for( int i = 0 ; i < info.size() ; i++ )
		{
			double dist = /*Strict*/Math.sqrt( (HumanPosition.x-info.get(i).get(1))*(HumanPosition.x-info.get(i).get(1)) +
					(HumanPosition.y-info.get(i).get(2))*(HumanPosition.y-info.get(i).get(2)) );
			if( dist <= info.get(i).get(0)+this.avoiDistance)
			{
				Vector2D temp = HumanPosition.subtract( new Vector2D( info.get(i).get(1), info.get(i).get(2)) );
				temp = temp.normalize();
				away = away.add( temp ); 
			}
		}
		return away.normalize();
	}


	
	/************************************
	 * Get goal to the exit point 
	 * @param bd
	 * @return
	 *************************************/
	private Vector2D getGoal(Crowd bd) {
		//goal is to go the exit gate 
		Vector2D goal = new Vector2D(0,0);
		
		int yGoal =0;
		double mid = (Crowd.doorLeft + Crowd.doorRight)/2;
		int doorLen = (int)(Crowd.doorRight - Crowd.doorLeft);

		boolean upper =false;
		boolean lower =false;


		Vector2D upperDoor = new Vector2D(mid,Crowd.yMinB);  //upper door center
		Vector2D lowerDoor = new Vector2D(mid,Crowd.yMaxB);	 //lower door center

		//check if human is in visibility range of upper door
		double huDistupper = Math.sqrt( distanceSquared(upperDoor, HumanPosition)  );
		double huDistlower = Math.sqrt( distanceSquared(lowerDoor, HumanPosition)  );

		if(huDistupper <= this.UpperVisibility)
			upper = true;
		if(huDistlower <= this.lowerVisibility)
			lower = true;

		if(!(upper || lower))
		{
			lower = bd.random.nextBoolean();
			upper = !lower;
			//	goal = towardsFlockCenterOfMass(bd).amplify(0.5);
		}


		if(upper)
		{
			//give upper door as goal
			yGoal = (int) (Crowd.yMinB+bd.random.nextInt(10)-bd.random.nextInt(7));

			goal = goal.add(( new Vector2D(Crowd.doorLeft+bd.random.nextInt(doorLen)-0.5 , 
					bd.random.nextInt(yGoal ) ) ));

		}
		if(lower)
		{
			//give lower door as goal
			yGoal = (int) (Crowd.yMaxB-bd.random.nextInt(10)+bd.random.nextInt(7));

			goal = goal.add(( new Vector2D(Crowd.doorLeft+bd.random.nextInt(doorLen)-0.5 , 
					yGoal+(Crowd.YMAX-Crowd.yMaxB) ) ));


		}

		goal = goal.subtract(HumanPosition);

		return goal.normalize();
	}
	public String toString() { return "[" + System.identityHashCode(this) + "] Physical Pressure: " + getForce() + "\n Panic: "+getPanic(); }
	/************************************************************************************
	 * Step the simulation
	 ************************************************************************************/
	public void step(final SimState state) 
	{

		Crowd bd = (Crowd) state;
		{	//faster way to access the agent
			Double2D temp = new Double2D(x,y);  
			HumanPosition.x = x;
			HumanPosition.y = y;
			this.avoiDistance = this.diameter+2;
			this.UpperVisibility =(int)(Crowd.yMaxB-Crowd.yMinB)/2;
			this.lowerVisibility = (int) 5*this.diameter;;
			preprocessHumans( bd, temp, MAX_DISTANCE );
		}
		if(!this.dead)
		{
		Vector2D vel = new Vector2D( 0, 0 );

		//if you are inside the box get the goal amplified by 5.5 otherwise 7.5
		if ( (HumanPosition.y> Crowd.yMinB ) || HumanPosition.y<Crowd.yMaxB);
		vel = vel.add( getGoal(bd).amplify(5.5));

		//if you are near the exit and you can see so more eager to go to the goal
		if ( ( (Math.abs(HumanPosition.x -Crowd.doorLeft) <50) ||(Math.abs(HumanPosition.x -Crowd.doorRight)<50 ) ) 
				&&( (HumanPosition.y-Crowd.yMinB )<20  ||(Crowd.yMaxB-HumanPosition.y )<20) ) 
			vel = vel.add( getGoal(bd).amplify(7.5));

		//Avoid obstacles, If close to door avoid obstacle more
		if( (HumanPosition.x>Crowd.doorLeft && HumanPosition.x<Crowd.doorRight) 
				&&( (HumanPosition.y-Crowd.yMinB )<5 ) ||(Crowd.yMaxB-HumanPosition.y)<5)
			vel = vel.add( avoidObstacles(bd).amplify( 2.5 ) );
		else
			vel = vel.add( avoidObstacles(bd).amplify( 0.5 ) );

		//Add cohesion --Herding behavior
		vel = vel.add( towardsFlockCenterOfMass(bd).amplify(0.5) );

		//alignment added try to match the group speed
		vel = vel.add( matchFlockSpeed(bd).amplify(1.5) );

		//if close to door get away from close by human by a large factor
		if ( ( (Math.abs(HumanPosition.x -Crowd.doorLeft) <20) ||(Math.abs(HumanPosition.x -Crowd.doorRight)<20 ) ) 
				&&( (HumanPosition.y-Crowd.yMinB )<15 ) ||(Crowd.yMaxB-HumanPosition.y)<15)												  
			vel = vel.add( awayFromCloseBys(bd).amplify(2.5) );
		else
			vel = vel.add( awayFromCloseBys(bd).amplify(1) );

		//add a little randomness!!
		vel = vel.add(randomDirection(bd));

		double vl = vel.length();
		if( vl < MIN_VELOCITY ){
			vel =vel.add( towardsFlockCenterOfMass(bd).amplify(0.5) );
		}
		else if( vl > MAX_VELOCITY ){
			vel = vel.setLength( MAX_VELOCITY );
		}

		//Model for panic
		calculatePanic(bd,this);					//call to get panic


		if(this.panic >=1)							// A highly panic situation
		{
			//forget everything and do what others do(high mass behavior)
			//washed the perception
			vel = new Vector2D( 0, 0 ); 
			vel = vel.add( towardsFlockCenterOfMass(bd).amplify(0.5) );
			 

			//mean all the surrounding panic and assign it to the person
			if( nearbyHuman != null )
			{
				double mean = 0;
				int n = 1;

				for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
				{
					mean += ((Human)(nearbyHuman.objs[i])).getPanic();
					n++;

				}
				this.panic = mean/n;
			}
			 
			 
		}
		//if panic is not madness and have not overcome than it will actually affect the movement.
		//and perceived reality.
		if(this.panic <=0.06 && this.panic>=0.01 )
			ADJUSTMENT_RATE = this.panic;

		vel = new Vector2D( (1-ADJUSTMENT_RATE)*velocity.x + ADJUSTMENT_RATE*vel.x,
				(1-ADJUSTMENT_RATE)*velocity.y + ADJUSTMENT_RATE*vel.y );

		if( vl < MIN_VELOCITY ){
			vel =vel.add( towardsFlockCenterOfMass(bd).amplify(0.5) );
		}

		//tweaking  away with close by more!!

		vel = vel.add(awayFromCloseBys(bd).amplify(0.05));


		velocity = vel;
		dvel.x  =vel.x;
		dvel.y = vel.y;

		Double2D desiredPosition = new Double2D( HumanPosition.x+vel.x*Crowd.TIMESTEP,
				HumanPosition.y+vel.y*Crowd.TIMESTEP );

		if(isValidMove(bd,this, desiredPosition) )
		{
			bd.setObjectLocation( this, desiredPosition );

			if(desiredPosition.y <Crowd.yMinB-5)
				Crowd.upDoorCount++;

			if(desiredPosition.y > Crowd.yMaxB+5)
				Crowd.lwDoorCount++;
		}
		//remove the agents which are outside the box now!!
		if(HumanPosition.y < Crowd.yMinB-5  || HumanPosition.y>Crowd.yMaxB+5 )
		{	 
			Crowd.HumansEnvironment.remove(this);
		}

		if(trigger >5000) 
			calculateSocialForce();
		trigger++;
	}
}
/*******************************************************
 *Panic Model
 * 
 *******************************************************/
private  void calculatePanic(Crowd bd, Human human) {

	double smoother = 0.0001;
	double soothDistance = 5*this.diameter;

	double midUp = (Crowd.doorLeft + Crowd.doorRight)/2;
	double midLow = (Crowd.doorLeft + Crowd.doorRight)/2;

	double panicDistance;
	int count = 0;
	Vector2D upperDoor = new Vector2D(midUp,Crowd.yMinB);  //upper door center
	Vector2D lowerDoor = new Vector2D(midLow,Crowd.yMaxB);	 //lower door center

	//check if human is in visibility range of upper door
	double huDistupper = Math.sqrt( distanceSquared(upperDoor, HumanPosition)  );
	double huDistlower = Math.sqrt( distanceSquared(lowerDoor, HumanPosition)  );


	if(huDistupper< huDistlower)
		panicDistance = huDistupper;
	else
		panicDistance = huDistlower;

	if(panicDistance <= soothDistance)
		human.panic -= (soothDistance -panicDistance)*smoother;

	//near by humans
	if( nearbyHuman != null && nearbyHuman.numObjs <5)
		human.panic +=(5-nearbyHuman.numObjs)*smoother;


	//neighbor velocity
	if( nearbyHuman != null )
	{	

		Vector2D mean = new Vector2D( 0, 0 );
		int n = 0;
		for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
		{		
			mean = mean.add( ((Human)(nearbyHuman.objs[i])).velocity );
			n++;
			if ( ((Human)(nearbyHuman.objs[i])).dead ==true    )
				count+=1;
		}
		double panicVelocity = (mean.length()/n) - velocity.length();
		human.panic +=panicVelocity*0.01;
	}
	if(count !=0)
		human.panic += count*smoother;

	human.panic += (MIN_VELOCITY -velocity.length())*0.01;

	//	System.out.println("Panic"+human.panic);
	/*
	if(human.panic>0.06)
	{ 
		human.panic = 0.06;
	}
	 */	
	if(human.panic<0.001)
		human.panic = 0.01;

}


/*********************************************************************
 * Check if the move is valid or not then only allow it.
 * It contains the personal space factor thing also!!
 * @param crowd
 * @param human
 * @param newLoc
 * @return
 ***************************************************************************/
public boolean isValidMove( final Crowd crowd, Human human, final Double2D newLoc)
{

	if( nearbyHuman == null )
		return true;

	double dist = 0;
	//to avoid personal space or restrict other agent from entering inside the agent!!
	for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
	{
		if( nearbyHuman.objs[i] != this)

		{
			Human p = (Human)(nearbyHuman.objs[i]);
			dist = distanceSquared(newLoc.x,newLoc.y,p.x,p.y);
			if( ( human.getRadius()+p.getRadius() )  > Math.sqrt(dist))
			{
				Vector2D temp = HumanPosition.subtract(new Double2D(p.x,p.y));
				temp = temp.normalize();
				velocity = velocity.add(temp).amplify(0.2);
				//return false;				
			}				
		}
	}
	//if reached Door then allow to leave
	if( (newLoc.x> Crowd.doorLeft-human.diameter && newLoc.x< Crowd.doorRight+human.diameter)&& (y < Crowd.yMinB+7 || y >Crowd.yMaxB-7) ) //for two doors!!
		return true;


	//Mechanism to restrict them inside the box walls
	if(HumanPosition.x <Crowd.xMaxB && (HumanPosition.y<Crowd.yMinB+human.diameter ||HumanPosition.y>Crowd.yMaxB+human.diameter ))
	{
		return false;
	}
	else
	{
		// check walls
		if(newLoc.x > Crowd.xMaxB-human.diameter)
		{
			if (velocity.x > 0) velocity.x = -velocity.x*WALL_MULT;
			return false;
		}
		else if(newLoc.x < Crowd.xMinB+human.diameter)
		{
			if (velocity.x < 0) velocity.x = -velocity.x*WALL_MULT;
			return false;
		}
		else if(newLoc.y > Crowd.yMaxB-human.diameter)
		{
			if (velocity.y > 0) velocity.y = -velocity.y*WALL_MULT;
			return false;
		}
		else if(newLoc.y < Crowd.yMinB+human.diameter)
		{
			if (velocity.y < 0) velocity.y = -velocity.y*WALL_MULT;
			return false;
		}
	}
	// no collisions
	return true;
}

/*********************************************************************
 * How much deadly force is squeezing the human 
 *********************************************************************/
public void calculateSocialForce()
{
	Bag injuryFromHuman = new Bag();

	for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
	{
		Human p = (Human)(nearbyHuman.objs[i]);

		double distance  = Math.sqrt(distanceSquared(HumanPosition,p.HumanPosition));

		if(distance < this.diameter)
			injuryFromHuman.add(p);
	}

	//	System.out.println("size"+ injuryFromHuman.size());

	double Ri = this.getRadius();                     // Agent's radius
	double Mi = this.getMass();			//60-100 Kg
	double fijx = 0;    //  X-direction interaction force initialization
	double fijy = 0;    //  Y-direction interaction force initialization

	// #####################################################################
	// Update every time step for every neighbor
	// #####################################################################

	for (int i = 0; i < injuryFromHuman.numObjs; i++) {

		Human p = (Human)(injuryFromHuman.objs[i]);
		if (p.equals(this)) 
			continue;


		Vector2D tempPosition = p.HumanPosition;
		Vector2D tempVel = p.velocity;
		double Rj =  p.getRadius();
		
		//scaling every meter by 6 for mason units
		double Aj = 2000;        //2000;       	  	//Published value = 2000 N
		double Bj = 0.48;        //0.08*6           			 //Published value = 0.08 m
		double kj = 12000;       //12000;     			 //Published value = 120000 kg/s^2
		double kappaj = 4000;   //24000/6;     			 //Published value = 240000 kg/ms


		double Rij = (Ri + Rj);    
		double Dij = Math.sqrt(distanceSquared(HumanPosition,tempPosition));
		double RD = (Rij - Dij);    //mostly positive(that means agents are pushing)


		if(Dij == 0.0)
			Dij =0.001;

		double Nijx = (HumanPosition.x - tempPosition.x) / Dij;    			// Normal X-direction unit vector
		double Nijy = (HumanPosition.y - tempPosition.y) / Dij;    			// Normal Y-direction unit vector



		double Tijx = -Nijy;            				// Tangential X-direction unit vector
		double Tijy = Nijx;            					 // Tangential Y-direction unit vector
		double DeltaVtji = (tempVel.x - velocity.x) * Tijx + (tempVel.y - velocity.y) * Tijy;

		double fsr = Aj * Math.exp((RD / Bj));  // Social Repulsion force
		double fp = kj * g(RD);                 // Pushing force
		double ff = kappaj * g(RD) * DeltaVtji; // Friction force

		fijx += (fsr + fp) * Nijx + ff * Tijx;
		fijy += (fsr + fp) * Nijy + ff * Tijy;
	}

	//	System.out.println("fijx "+fijx);
	//		System.out.println("fijy "+fijy);

	fijx = fijx/Mi;
	fijy = fijy/Mi;
	Double2D force = new Double2D(fijx,fijy);
	deathForce = force.length();
	injuryFromHuman.clear();

	if(deathForce >850 )
		this.dead   = true;
}

public double getPanic() {

	return this.panic;
}



public Tuple2d getCurrentPosition() {
	Tuple2d t = new Tuple2d() {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;
	};
	t.setX(HumanPosition.x);
	t.setY(HumanPosition.y);
	return t;
}


public double getVelocity() {
	return velocity.length();
}

public double getRadius() {

	return this.diameter/2;
}

public double getMass() {
	return this.mass;
}

private double g(double x) {
	if (x >= 0) {
		return x;
	} else {
		return 0;
	}
}

public double getForce()
{
	return this.deathForce;
}

public void setPanic(double panic) {
	this.panic = panic;
}

public double getMaxSpeed() {
	// TODO Auto-generated method stub
	return 0;
}


}
