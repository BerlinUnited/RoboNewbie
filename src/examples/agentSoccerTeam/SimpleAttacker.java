/**
 * *****************************************************************************
 * RoboNewbie NaoTeam Humboldt
 *
 * @author Hans-Dieter Burkhard, 11.2.2015, 26.3.2015
 * @version 1.1
 *
 * The original decide method was written by Damyan Damyanov,Ivelin Rusev, 
 * and Petar Bilev during the course in Plovdiv 2014. 
 * With their very strong kick they won the course competition with only 
 * 40 sec to score. 
 * The method was partially adapted, improved, and polished for the use in the
 * SoccerTeam context.
 * *****************************************************************************
 */
package examples.agentSoccerTeam;

import agentIO.PerceptorInput;
import keyframeMotion.KeyframeMotion;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import util.Logger;

/**
 * This class implements the decisions of a player. The decide method was
 * adapted from Agent_SimppleSoccer. But instead pushing, the agent kicks the
 * ball towards the goal.
 */
public class SimpleAttacker extends Role {

    public boolean nearForKick = false;

    public SimpleAttacker(KeyframeMotion motion, PerceptorInput percIn, Logger log) {
        super(motion, percIn, log);
    }

    @Override
    public void decide() {

        final double TOLLERATED_DEVIATION = Math.toRadians(6);
        final double TOLLERATED_DISTANCE = 0.7; // in meters

        if (motion.ready()) 
        {
            double serverTime = percIn.getServerTime();
            // if the robot has fallen down
            if (percIn.getAcc().getZ() < 2) 
            {
                if (percIn.getAcc().getY() > 0) 
                {
                    motion.setStandUpFromBack();
                } else 
                {
                    motion.setRollOverToBack();
                }
                nearForKick = false;
            } // if the robot has the actual ball coordinates
            else if ((serverTime - ball.getTimeStamp()) < lookTime) 
            {
                Vector3D ballCoords = ball.getCoords();
//              log.log("2. robot has the actual ball coordinates, horizontal angle: " 
//                + Math.toDegrees(ballCoords.getAlpha()) 
//                + " distance: " + ballCoords.getNorm()) ;

                // if the ball is not in front of the robot
                if (Math.abs(ballCoords.getAlpha()) > TOLLERATED_DEVIATION) 
                {
//                  log.log("3. the ball is not in front of the robot. ") ;
                    if (robotIsWalking) 
                    {
                        motion.setStopWalking_Plovdiv2014();
                        robotIsWalking = false;
                    } else {
                        if (ballCoords.getAlpha() > 0) 
                        {
                            motion.setTurnLeftSmall();
                        } else 
                        {
                            motion.setTurnRightSmall();
                        }
                    }
                } // if the robot is far away from the ball
                else if (ballCoords.getNorm() > TOLLERATED_DISTANCE) 
                {
//                  log.log("3. the robot is far away from the ball.");
                    motion.setBadWalk_Plovdiv2014();
                    robotIsWalking = true;
                } // if the robot has the actual goal coordinates
                else if ((serverTime - oppGoalLPost.getTimeStamp() < lookTime)
                        && (serverTime - oppGoalRPost.getTimeStamp() < lookTime)) 
                {
//                  log.log("5. the robot has the actual goal coordinates");

                    // if the ball does not lie between the robot and the goal
                    if ((oppGoalLPost.getCoords().getAlpha() <= ballCoords.getAlpha())
                            || (oppGoalRPost.getCoords().getAlpha() >= ballCoords.getAlpha())) 
                    {
                        log.log("6. the ball does not lie between the robot and the goal");
                        if (robotIsWalking) {
                            motion.setStopWalking_Plovdiv2014();
                            robotIsWalking = false;
                        } else {
                            if (oppGoalLPost.getCoords().getAlpha() <= ballCoords.getAlpha()) 
                            {
                                motion.setSideStepLeft();
                            } else {
                                motion.setSideStepRight();
                            }
                        }
                    } // if the robot is close to a good kick position
                    else {
                        if (nearForKick) {
                            motion.setKick_Plovdiv2014();
                            robotIsWalking = false;
                            nearForKick = false;
                        } else 
                        {
                            if (robotIsWalking) 
                            {
                                motion.setStopWalking_Plovdiv2014();
                                robotIsWalking = false;
                            } else 
                            {
                                motion.setWalkForward();
                                robotIsWalking = true;  
                                nearForKick = true;
                            }
                        }
                    }
                } // if the robot cannot sense the goal coordinates from its actual position
                else 
                {
                    log.log("5. goal coordinates missing");
                    System.out.println("....etTurnLef-13");
                    motion.setTurnLeft();
                }
            } // if the robot cannot sense the ball coordinates from its actual position
            else 
            {
                motion.setTurnLeft();
                log.log("1. ball coordinates missing");
            }
        }
    }
}
