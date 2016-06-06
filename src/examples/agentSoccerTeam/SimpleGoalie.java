/**
 * *****************************************************************************
 * RoboNewbie NaoTeam Humboldt
 *
 * @author Hans-Dieter Burkhard, 11.2.2015
 * @version 1.1
 *
 * *****************************************************************************
 */


package examples.agentSoccerTeam;

import agentIO.PerceptorInput;
import keyframeMotion.KeyframeMotion;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import util.Logger;

/**
    * This class implements the decisions of a primitive goal keeper.  
    * The Goalie walks to the ball if it becomes near. 
    * No return to the goal.
 */
public class SimpleGoalie extends Role {

    public SimpleGoalie(KeyframeMotion motion, PerceptorInput percIn, Logger log) {

        super(motion, percIn, log);
    }

    @Override
    public void decide() {

        double TOLLERATED_DEVIATION = Math.toRadians(6);
        double LIMIT_FOR_BALL = 2; // in meters

        if (motion.ready()) {

            double serverTime = percIn.getServerTime();

            // if the robot has fallen down
            if (percIn.getAcc().getZ() < 2) {
                if (percIn.getAcc().getY() > 0) {
                    motion.setStandUpFromBack();
                } else {
                    motion.setRollOverToBack();
                }
            } // if the robot has the actual ball coordinates
            else if ((serverTime - ball.getTimeStamp()) < lookTime) {

                Vector3D ballCoords = ball.getCoords();
                // if the ball is not in front of the robot
                if (Math.abs(ballCoords.getAlpha()) > TOLLERATED_DEVIATION) {
                    if (robotIsWalking) {
                        motion.setStopWalking();
                        robotIsWalking = false;
                    } else {
                        if (ballCoords.getAlpha() > 0) {
                            motion.setTurnLeftSmall();
                        } else {
                            motion.setTurnRightSmall();
                        }
                    }
                } // if the ball is close to the robot
                else if (ballCoords.getNorm() < LIMIT_FOR_BALL) {
                    motion.setWalkForward();
                    robotIsWalking = true;
                }
            }
        }
    }
}
