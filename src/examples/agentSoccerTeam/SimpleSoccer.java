/**
 * *****************************************************************************
 * RoboNewbie NaoTeam Humboldt
 *
 * @author Hans-Dieter Burkhard, 11.2.2015
 * @version 1.1
 *
 * The program is based on the class examples.agentSimpleSoccer.SoccerThinking.
 * *****************************************************************************
 */

package examples.agentSoccerTeam;

import agentIO.PerceptorInput;
import keyframeMotion.KeyframeMotion;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import util.Logger;

/**
 * This class replicates the decision in the introductory example Agent_simpleSoccer. 
 * It implements a simple behavior to push the ball to the opponent goal. 
 * The behavior works in details as follows: 
 * The decision of a new move is made every time, when the robot does not 
 * execute any motion (but the head motion to sense the field objects). It 
 * depends on conditions, that must be fulfilled sequentially. 
 * 1) Is the robot upright? If is has fallen down, it should perform a stand up
 * motion.
 * 2) Does the robot have the actual ball coordinates relative to itself? If 
 * not, the robot should turn to change its position to get a new perspective on 
 * the field. 
 * 3) Is the ball exactly in front of the robot? If not, turn to the ball. 
 * 4) Is the ball near enough to push it? If not, walk forward to it. 
 * 5) Does the robot have the actual goal coordinates relative to itself? If
 * not, it should turn as in 2) .
 * 6) Does the ball lie between the robot and the goal? If not do side steps to
 * reach this position.
 * 7) If all of the conditions above are fulfilled, then the robot is in a 
 * good position to push the ball forward towards the goal. 
 */
public class SimpleSoccer extends Role {

    public SimpleSoccer(KeyframeMotion motion, PerceptorInput percIn, Logger log) {

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
                        motion.setStopWalking();
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
                    motion.setWalkForward();
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
                            motion.setStopWalking();
                            robotIsWalking = false;
                        } else {
                            if (oppGoalLPost.getCoords().getAlpha() <= ballCoords.getAlpha()) {
                                motion.setSideStepLeft();
                            } else {
                                motion.setSideStepRight();
                            }
                        }
                    } // if the robot is in a good dribbling position
                    else {
//            log.log("7. the robot is in a good dribbling position");
                        motion.setWalkForward();
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
