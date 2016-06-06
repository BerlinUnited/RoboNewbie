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
import agentIO.EffectorOutput;
import keyframeMotion.KeyframeMotion;
import localFieldView.BallModel;
import localFieldView.GoalPostModel;
import util.Logger;

/**
 * This class is the super class for all individual roles of players.
 */


public abstract class Role {

    public PerceptorInput percIn;
    public EffectorOutput effOut;
    public KeyframeMotion motion;
    public Logger log;
    public BallModel ball;
    public GoalPostModel oppGoalLPost, oppGoalRPost, ownGoalLPost, ownGoalRPost;
    public double lookTime;
    //parameters to pass information between cycles
    static boolean robotIsWalking;
    static int count;

    public Role(KeyframeMotion motion, PerceptorInput percIn, Logger log) {

        this.percIn = percIn;
        this.motion = motion;
        this.log = log;

        ball = SoccerTeamThinking.ball;
        lookTime = SoccerTeamThinking.lookTime;

        oppGoalLPost = SoccerTeamThinking.oppGoalLPost;
        oppGoalRPost = SoccerTeamThinking.oppGoalRPost;
        ownGoalLPost = SoccerTeamThinking.ownGoalLPost;
        ownGoalRPost = SoccerTeamThinking.ownGoalRPost;

        robotIsWalking = false;
        count = 0;
    }

    public void decide() {
    }
}
