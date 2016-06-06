/**
 * *****************************************************************************
 * RoboNewbie NaoTeam Humboldt
 *
 * @author Hans-Dieter Burkhard, 11.2.2015, 12.3.2015
 * @version 1.1
 *
 * *****************************************************************************
 */
package examples.agentSoccerTeam;

import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import directMotion.LookAroundMotion;
import java.util.HashMap;
import keyframeMotion.KeyframeMotion;
import localFieldView.BallModel;
import localFieldView.GoalPostModel;
import localFieldView.LocalFieldView;
import util.FieldConsts.GoalPostID;
import util.Logger;

/**
 * The decide method for the think part in class Agent_SoccerTeam depends on the
 * role of a player. The roles are assigned according to the player numbers ("1"
 * for the goalie etc.). The related decide methods are implement in special
 * classes (inheriting from class Role).
 *
 * The assignments of the roles are performed here: The numbered roles have to
 * be declared, the classes have to be instantiated, and the related
 * decide-procedures are then called in each cycle.
 *
 * The player numbers are usually assigned by the server (if id = 0 was set in
 * Agent_SoccerTeam). The related beam positions were defined in the class
 * util.BeamPoses.
 *
 * All parameters determining the identity (teamSide, teamName, finalRobotID,
 * finalBeamCoordsX, finalBeamCoordsY, finalBeamCoordsRot) are contained in
 * ServerCommunication.
 */
public class SoccerTeamThinking {

    Logger log;
    PerceptorInput percIn;
    LocalFieldView localView;
    KeyframeMotion motion;
    static BallModel ball;
    static GoalPostModel oppGoalLPost, oppGoalRPost, ownGoalLPost, ownGoalRPost;
    static String playerNumber;
    static double lookTime;
    /**
     * Declaration of roles.#
     *
     * Must be updated for new or changing roles.
     */
    SimpleGoalie role1;
    SimpleSoccer_Plovdiv2014 role2;
    SimpleSoccer role3;
    SimpleSoccer_withKick role4;
    SimpleSoccer role_d;

    /**
     * Constructor.
     *
     * @param All passed object must be already initalized, and correctly
     * synchronized with the simulation server. None of the arguments can be
     * null.
     */
    public SoccerTeamThinking(PerceptorInput percIn, LocalFieldView localView,
            KeyframeMotion kfMotion, Logger log, ServerCommunication sc) {
        this.percIn = percIn;
        this.localView = localView;
        this.motion = kfMotion;
        //this.motion.setLogging(true);
        this.log = log;

        /**
         * The decide procedures in the different roles need the coordinates of
         * the ball and the goals for orientation.
         */
        ball = this.localView.getBall();

        HashMap<GoalPostID, GoalPostModel> goalPosts = this.localView.getGoals();
        /**
         * The opponent goal is the right one (with oppGoalLPost = G1R) if the
         * team starts from the left side. It is the left one (with oppGoalLPost
         * = G2R from the players view) if the team starts on the right side.
         * The teamside is inferred after initialization in ServerCommunication.
         */
        if (sc.teamSide.equals("left")) {

            oppGoalLPost = goalPosts.get(GoalPostID.G1R);
            oppGoalRPost = goalPosts.get(GoalPostID.G2R);
            ownGoalLPost = goalPosts.get(GoalPostID.G1L);
            ownGoalRPost = goalPosts.get(GoalPostID.G2L);
        } else {
            oppGoalLPost = goalPosts.get(GoalPostID.G2L);
            oppGoalRPost = goalPosts.get(GoalPostID.G1L);
            ownGoalLPost = goalPosts.get(GoalPostID.G1R);
            ownGoalRPost = goalPosts.get(GoalPostID.G2R);
        }

        playerNumber = sc.finalRobotID;
        lookTime = LookAroundMotion.LOOK_TIME;


        /**
         * Assignment of decide methods to the roles.
         * 
         * They are assigned according to the player numbers. Decide methods are
         * implemented by the related role classes. They are called in method
         * decide below. 
         * 
         * If another role is assigned to a number, the declaration above must
         * be changed accordingly.
         */
        switch (playerNumber) {
            case "1":
                role1 = new SimpleGoalie(motion, percIn, log);
                break;
            case "2":
                role2 = new SimpleSoccer_Plovdiv2014(motion, percIn, log);
                break;
            case "3":
                role3 = new SimpleSoccer(motion, percIn, log);
                break;
            case "4":
                role4 = new SimpleSoccer_withKick(motion, percIn, log);
                break;

            /**
             * // case "5": break; // case "6": break; // case "7": break; //
             * case "8": break; // case "9": break; // case "10": break; // case
             * "11": break;
             *
             *
             *
             */
            default:
                role_d = new SimpleSoccer(motion, percIn, log);
        }
    }

    /**
     * To use the decide procedure for number n, the related
     * role for player n must have been declared and specified above. 
     * Instead you can use the default role.
     */
    public void decide() {
        switch (playerNumber) {
            case "1":
                role1.decide();
                break;
            case "2":
                role2.decide();
                break;
            case "3":
                role3.decide();
                break;
            case "4":
                role4.decide();
                break;
            /**
             * no role specified up to now:
             *
             * case "5": role5.decide(); break; case "6": role6.decide(); break;
             * case "7": role7.decide(); break; case "8": role8.decide(); break;
             * case "9": role9.decide(); break; case "10": role10.decide();
             * break; case "11": role11.decide(); break;
             *
             */
            default:
                role_d.decide();
        }
    }
}
