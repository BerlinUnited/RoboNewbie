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
   * This class implements the decisions of a player.
   * The decide method was adapted from Agent_SimppleSoccer. But instead pushing,
   * the agent kicks the ball towards the goal.  
   * It uses better keyframes for the walk and a keyframe for kick.
   * Both were originally implemented by Luka Unuk from Rijeka. His program 
   * won the Rijeka course competition 2013 with 57 sec to score.  
   * The class uses the keyframe StopWalking_Plovdiv2014.
   * The program needs some calibrations ...
 */
public class SimpleSoccer_withKick extends Role{
   
    public SimpleSoccer_withKick(KeyframeMotion motion, PerceptorInput percIn, Logger log){ 
        
      super(motion, percIn, log); 
    }
    
    @Override
    public void decide() {
        double TOLLERATED_DEVIATION = Math.toRadians(6);
        double TOLLERATED_DISTANCE = 0.6; // in meters

        if (motion.ready()) {

            //log.log("new decision");
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
//        log.log("2. robot has the actual ball coordinates, horizontal angle: " 
//                + Math.toDegrees(ballCoords.getAlpha()) 
//                + " distance: " + ballCoords.getNorm()) ;

                // if the ball is not in front of the robot
                if (Math.abs(ballCoords.getAlpha()) > TOLLERATED_DEVIATION) {
//          log.log("3. the ball is not in front of the robot. ") ;
                    if (robotIsWalking) {
                        motion.setStopWalking_Plovdiv2014();
                        robotIsWalking = false;
                    } else {
                        if (ballCoords.getAlpha() > 0) {
                            motion.setTurnLeftSmall();
                        } else {
                            motion.setTurnRightSmall();
                        }
                    }
                } // if the robot is far away from the ball
                else if (ballCoords.getNorm() > TOLLERATED_DISTANCE) {
//          log.log("3. the robot is far away from the ball.");        
                    motion.setWalkForward_Rijeka2013();
                    robotIsWalking = true;
                } // if the robot has the actual goal coordinates
                else if ((serverTime - oppGoalLPost.getTimeStamp() < lookTime)
                        && (serverTime - oppGoalRPost.getTimeStamp() < lookTime)) {
//          log.log("5. the robot has the actual goal coordinates");

                    // if the ball does not lie between the robot and the goal
                    if ((oppGoalLPost.getCoords().getAlpha() <= ballCoords.getAlpha())
                            || (oppGoalRPost.getCoords().getAlpha() >= ballCoords.getAlpha())) {
//            log.log("6. the ball does not lie between the robot and the goal");
                        if (robotIsWalking) {
                            motion.setStopWalking_Plovdiv2014();
                            robotIsWalking = false;
                        } else {
                            if (oppGoalLPost.getCoords().getAlpha() <= ballCoords.getAlpha()) {
                                motion.setSideStepLeft();
                            } else {
                                motion.setSideStepRight();
                            }
                        }
                    } // if the robot is in a good scoring position
                    else {
//            log.log("7. the robot is in a good scoring position");
                        motion.setKick_Rijeka2013();
                        robotIsWalking = true;
                    }
                } // if the robot cannot sense the goal coordinates from its actual position
                else {
//          log.log("5. goal coordinates missing");
                    motion.setTurnLeft();
                }
            } // if the robot cannot sense the ball coordinates from its actual position
            else {
                motion.setTurnLeft();
//        log.log("1. ball coordinates missing");
            }
        }
    }

}
