
package human.obstacle;


import javax.vecmath.Vector2d;
import sim.util.Bag;

/**
 *
 * @author michaellees
 *
 * The interface which needs to implemented by all classes that implement 
 * collision detection algorithms
 *
 */
public interface VelocityCalculator {

       public Vector2d calculateVelocity(Human me,
            Bag neighbors, Bag obses, Vector2d preferredVelocity, double timeStep);
}
