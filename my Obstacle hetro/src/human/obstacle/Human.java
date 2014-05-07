package human.obstacle;

import sim.engine.SimState;
import sim.engine.Steppable;
 
import java.awt.*;
import java.util.ArrayList;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

import sim.portrayal.*;
import sim.portrayal.simple.OvalPortrayal2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;


public class Human  extends OvalPortrayal2D implements Steppable {

	private static final long serialVersionUID = 1;
	double diameter;
	private double mass;
	public static int trigger = 0;

	public static final double CENTROID_DISTANCE = 20 ;
	public  double avoiDistance = 10;
	public static final double COPY_SPEED_DISTANCE = 10;


	public static double MAX_DISTANCE = /*Strict*/Math.max( CENTROID_DISTANCE,
			/*Strict*/Math.max( 6,
					COPY_SPEED_DISTANCE ) );

	public static  double ADJUSTMENT_RATE = 0.05;
	public static final double MIN_VELOCITY = 0.35;
	public static final double MAX_VELOCITY = 0.95;


	public double panic =0.05 ;	//Trigger PANIC

	//if Panic it will be very less 0.02 else 0.5
	public static double WALL_MULT = 0.4;		//repulsion from wall and other immobile obstacle.

	public  double UpperVisibility ;  //visibility of the upper door  
	public  double lowerVisibility;   //visibility of the  lower door

	//All the multipliers
	//		private double goalMultiplier;
	//		private double obsMultiplier;
	//		private double cohesionMultiplier;
	//		private double alignMultiplier;
	//		private double seprationMultiplier;
	//		

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
	Bag nearbyObst;
	double[] distSqrTo;
	double[] distSqrToObs;
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
		nearbyHuman = state.HumansEnvironment.getNeighborsWithinDistance( pos, distance );
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
	/**************************************************************
	 * get nearBy Obstacles and position of each Obstacle
	 * @param state
	 * @param pos
	 * @param distance
	 *************************************************************/
	void preprocessObstacles( final Crowd state, Double2D pos, double distance )
	{
		nearbyObst = Crowd.obstaclesEnvironment.getNeighborsWithinDistance( pos, distance );
		if( nearbyObst == null )
		{
			return;
		}

		/*	
		distSqrToObs = new double[nearbyObst.numObjs];
		for( int i = 0 ; i < nearbyObst.numObjs ; i++ )
		{
			Obstacle o = (Obstacle)(nearbyObst.objs[i]); 
			Double2D oLoc = state.obstaclesEnvironment.getObjectLocationAsDouble2D(o);
			distSqrToObs[i] = distanceSquared(
					pos.x,pos.y,oLoc.x,oLoc.y);
		}
		 */}

	protected Vector2D HumanPosition = new Vector2D( 0, 0 );
	protected Vector2D previousPosition = new Vector2D( 0, 0 );

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
	boolean dead = false;
	private double deathForce;
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
	private Vector2D getExit(Crowd bd) {

		//double mid = (Crowd.doorLeft + Crowd.doorRight)/2;
		int doorLen = (int)(Crowd.doorRight - Crowd.doorLeft);

		int yGoal = (int) (Crowd.yMinB+bd.random.nextInt(10)-bd.random.nextInt(10));

		Vector2D goal = ( new Vector2D(Crowd.doorLeft+bd.random.nextInt(doorLen)-0.5 , 
				bd.random.nextInt(yGoal )) );
		goal = goal.subtract(HumanPosition);

		return goal.normalize();
	}
	/************************************
	 * Get goal near to the exit point 
	 * @param bd
	 * @return
	 *************************************/
	private Vector2D getGoal(Crowd bd) {
		//goal is to go the exit gate 

		int yGoal = (int)Crowd.yMinB + bd.random.nextInt(30);
		Vector2D goal =  new Vector2D(HumanPosition.x+bd.random.nextInt(30)-bd.random.nextInt(30), 
				yGoal) ;

		goal = goal.subtract(HumanPosition);

		return goal.normalize();
	}
	public String toString() { return "[" + System.identityHashCode(this) + "] Social Force: " + getForce() + "\n Panic: "+getPanic(); }
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
			preprocessHumans( bd, temp, MAX_DISTANCE );
			preprocessObstacles(bd, temp, 40);
		}

		if(!this.dead )
		{

		Vector2D vel = new Vector2D( 0, 0 );

		//if you are inside the box get the goal amplified by 5.5 otherwise 7.5
		if(HumanPosition.y> Crowd.yMinB )
			vel = vel.add( getGoal(bd).amplify(4.5));

		//if you are near the exit and you can see so more eager to go to the exit
		if ( ( (Math.abs(HumanPosition.x -Crowd.doorLeft) <50) ||(Math.abs(HumanPosition.x -Crowd.doorRight)<50 ) ) 
				&&( (HumanPosition.y-Crowd.yMinB )<20 ) ) 
		{
			vel = new Vector2D( 0, 0 );
			vel = vel.add( getExit(bd).amplify(5.5));
		}

		//Avoid obstacles, If close to door avoid obstacle more
		vel = vel.add( avoidObstacles(bd).amplify( 3.5 ) );


		//Add cohesion --Herding behavior
		vel = vel.add( towardsFlockCenterOfMass(bd).amplify(1.5) );


		//alignment added try to match the group speed
		vel = vel.add( matchFlockSpeed(bd).amplify(1.5) );


		//if close to door get away from close by human by a large factor
		if ( ( (Math.abs(HumanPosition.x -Crowd.doorLeft) <20) ||(Math.abs(HumanPosition.x -Crowd.doorRight)<20 ) ) 
				&&( (HumanPosition.y-Crowd.yMinB )<15 ) ) 
			vel = vel.add( awayFromCloseBys(bd).amplify(2.5) );
		else
			vel = vel.add( awayFromCloseBys(bd).amplify(1) );

		//add a little randomness!!
		vel = vel.add(randomDirection(bd).amplify(1.5));


		//remove the agents which are outside the box now!!
		if(HumanPosition.y < Crowd.yMinB)
			bd.HumansEnvironment.remove(this);


		double vl = vel.length();
		if( vl < MIN_VELOCITY ){
			vel = vel.setLength( MIN_VELOCITY );
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
			//	vel = new Vector2D( 0, 0 ); 
			vel = vel.add( towardsFlockCenterOfMass(bd).amplify(0.2) );
			//this.panic = 0.01;

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
			else
				this.panic = 0.01;
		}
		//if panic is not madness and have not overcome than it will actually affect the movement.
		//and perceived reality.
		if(this.panic <=0.06 && this.panic>=0.01 )
			ADJUSTMENT_RATE = this.panic;

		vel = new Vector2D( (1-ADJUSTMENT_RATE)*velocity.x + ADJUSTMENT_RATE*vel.x,
				(1-ADJUSTMENT_RATE)*velocity.y + ADJUSTMENT_RATE*vel.y );

		if( vl < MIN_VELOCITY ){
			vel = vel.setLength( MIN_VELOCITY );
		}

		//remove the agents which are outside the box now!!
		if(HumanPosition.y < Crowd.yMinB)
			Crowd.HumansEnvironment.remove(this);



		//tweaking with away with close by more!!
		if ( ( (Math.abs(HumanPosition.x -Crowd.doorLeft) <50) ||(Math.abs(HumanPosition.x -Crowd.doorRight)<50 ) ) 
				&&( (HumanPosition.y-Crowd.yMinB )<25 ) ) 
			vel = vel.add(awayFromCloseBys(bd).amplify(0.07));
		else
			vel = vel.add(awayFromCloseBys(bd).amplify(0.05));

		velocity = vel;
		dvel.x  =vel.x;
		dvel.y = vel.y;

		Double2D desiredPosition = new Double2D( HumanPosition.x+vel.x*Crowd.TIMESTEP,
				HumanPosition.y+vel.y*Crowd.TIMESTEP );

		if(isValidMove(bd,this, desiredPosition) )
			bd.setObjectLocation( this, desiredPosition );

		//remove the agents which are outside the box now!!
		if(HumanPosition.y < Crowd.yMinB-5)
			Crowd.HumansEnvironment.remove(this);

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
		double soothDistance = 10*this.diameter;

		Double2D doorPos = new Double2D(Crowd.doorMid,Crowd.yMinB);

		//distance
		double panicDistance = Math.sqrt(distanceSquared(HumanPosition, doorPos));
		if(panicDistance <= soothDistance)
			human.panic -= (soothDistance -panicDistance)*smoother;

		//near by humans
		if( nearbyHuman != null && nearbyHuman.numObjs <5)
			human.panic +=(5-nearbyHuman.numObjs)*smoother;


		//if in nearby humans you found dead panic will increase a lot!!


		//neighbor velocity
		if( nearbyHuman != null )
		{	
			Vector2D mean = new Vector2D( 0, 0 );
			int n = 0;
			for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
			{		
				mean = mean.add( ((Human)(nearbyHuman.objs[i])).velocity );
				n++;
			}
			double panicVelocity = (mean.length()/n) - velocity.length();
			human.panic +=panicVelocity*0.01;
		}


		human.panic += (MIN_VELOCITY -velocity.length())*0.01;

		//	System.out.println("Panic"+human.panic);


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
	public boolean isValidMove( final Crowd crowd, Human human, Double2D newLoc)
	{

		if( nearbyHuman == null && nearbyObst == null)
			return true;

		double dist = 0;
		double dist2 = 0;

		//to avoid personal space or restrict other agent from entering inside the agent!!
		for( int i = 0 ; i < nearbyHuman.numObjs ; i++ )
		{
			if( nearbyHuman.objs[i] != this)

			{
				Human p = (Human)(nearbyHuman.objs[i]);
				dist = distanceSquared(newLoc.x,newLoc.y,p.x,p.y);

				if( ( human.getRadius()+p.getRadius() ) > Math.sqrt(dist))
				{
					Vector2D temp = HumanPosition.subtract(new Double2D(p.x,p.y));
					temp = temp.normalize();
					velocity = velocity.add(temp).amplify(0.20);
					//return false;				
				}	
				//return false;				
			}
		}
		//avoid agent obstacle collision
		for( int j = 0 ; j < nearbyObst.numObjs ; j++ )
		{
			Obstacle o = (Obstacle)(nearbyObst.objs[j]); 
			Double2D oLoc = Crowd.obstaclesEnvironment.getObjectLocationAsDouble2D(o);

			dist = distanceSquared(newLoc.x,newLoc.y,oLoc.x,oLoc.y);
			if( o.diameter/2 + human.getRadius() +1.5 >Math.sqrt(dist)   )
			{
				Vector2D temp = HumanPosition.subtract( new Vector2D( oLoc.x,oLoc.y ) );
				temp = temp.normalize();
				velocity = velocity.add(temp).amplify(0.4);
				return false;
			}
		}

		//if reached Door then allow to leave
		if( (newLoc.x> Crowd.doorLeft-3 && newLoc.x< Crowd.doorRight+3)&& (y < Crowd.yMinB+7 ) )
			return true;


		//Mechanism to restrict then inside the box walls
		if(HumanPosition.x <Crowd.xMaxB && HumanPosition.y<Crowd.yMinB+this.diameter)
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

		if(deathForce >950 )
			this.dead  = true;

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





}