/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Heinrich Mellmann
* @version 1.1
*******************************************************************************/

package examples.parallelKinematikWalk;

import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import motion.SimplePositionControl;
import util.Logger;
import util.RobotConsts;



/**
 * This agent shows the structure of an agent, that does something sensible:
 * it walks into the direction of the ball.
 */
public class Agent_ParallelTest 
{

  public static void main(String args[]) {
    
    Agent_ParallelTest agent = new Agent_ParallelTest();
    agent.run();
    
    agent.log.printLog();
    System.out.println("Agent stopped.");
  }
  
  private final Logger log;
  private final ServerCommunication sc;
  private final PerceptorInput in;
  private final EffectorOutput out;
  private final SimplePositionControl positionControl;
  
  public Agent_ParallelTest() 
  {  
    sc = new ServerCommunication();
    sc.initRobot("1", "Parallel Kinematik", -1, 0, -90);
    
    in  = new PerceptorInput(sc);
    out = new EffectorOutput(sc);
    log = new Logger();
    positionControl = new SimplePositionControl(in, out);
  }
  
  
  /**
   * Main loop of the agent program, where it is synchronized with the 
   * simulation server. 
   */
  public void run()
  {  
    Pose left = new Pose();
    Pose right = new Pose();
    
    double lastServerTime = -1;
    double t = 0;
    
    // walk parameters
    double speed = 5.0;
    
    double body_shift = 0.2;

    double body_height = -0.7;
    double step_height = -0.4;
    double step_length_x = 0.0;
    double step_length_y = 0.1;
    
    while(true)
    {
        in.update();
        
        // initialize at the first run
        if(lastServerTime == -1) {
            lastServerTime = in.getServerTime();
        }
        
        // time since the last execution
        double timeDelta = in.getServerTime() - lastServerTime;
        lastServerTime = in.getServerTime();        

        t += speed*timeDelta;
            
        double y = body_shift * Math.sin(t);
        double z = body_height + step_height * (1.0-Math.cos(2.0*t))*0.5;

        double step_left_x = step_length_x * Math.sin(t + Math.PI*0.5);
        double step_left_y = step_length_y * (1+Math.sin(t + Math.PI*0.5))*0.5;        
        
        // calculate pose for the left foot
        left.x = step_left_x;
        left.y = y + step_left_y;
        left.z = (y < 0)?z:body_height;
                
        double step_right_x = step_length_x * Math.sin(t + Math.PI*1.5);
        double step_right_y = step_length_y * (1.0+Math.sin(t + Math.PI*1.5))*0.5;
        
        // calculate pose for the right foot
        right.x = step_right_x;
        right.y = y + step_right_y;
        right.z = (y > 0)?z:body_height;
                        
        // calculate the leg joint angles according to the parallel kinematic
        double[] jointData = calculateJoints(left, right);
                
        positionControl.setAllJointPositions(jointData);
        positionControl.update(); 
        out.sendAgentMessage();
    }
  }
    

  private class Pose {
      // foot position
      double x;
      double y;
      double z;
      
      // foot rotation
      double ax;
      double ay;
      double az;
  }
  
  private double[] calculateJoints(Pose poseLeft, Pose poseRight) 
  {
    double[] jointData = new double[RobotConsts.JointsCount];

    // symmetric control
    double az = (poseRight.az + poseLeft.az)*0.5;
            
    double left_ay = poseLeft.ay - az*0.5;
    double right_ay = poseLeft.ay - az*0.5;
    
    // left leg
    // y - axis
    jointData[RobotConsts.LeftHipPitch]  = -0.5 * poseLeft.z + poseLeft.x - (left_ay / 4);
    jointData[RobotConsts.LeftKneePitch] =        poseLeft.z              - (left_ay / 2);
    jointData[RobotConsts.LeftFootPitch] = -0.5 * poseLeft.z - poseLeft.x - (left_ay / 4);

    // x - axis
    jointData[RobotConsts.LeftHipRoll]   = -poseLeft.y + (poseLeft.ax / 2);
    jointData[RobotConsts.LeftFootRoll]  =  poseLeft.y + (poseLeft.ax / 2);

    
    // right leg
    // y - axis
    jointData[RobotConsts.RightHipPitch]  = -0.5 * poseRight.z + poseRight.x - (right_ay / 4);
    jointData[RobotConsts.RightKneePitch] =        poseRight.z               - (right_ay / 2);
    jointData[RobotConsts.RightFootPitch] = -0.5 * poseRight.z - poseRight.x - (right_ay / 4);

    // x - axis
    jointData[RobotConsts.RightHipRoll]   = -poseRight.y + (poseRight.ax / 2);
    jointData[RobotConsts.RightFootRoll]  =  poseRight.y + (poseRight.ax / 2);

    
    // z-axis
    jointData[RobotConsts.RightHipYawPitch] = az;
    jointData[RobotConsts.LeftHipYawPitch] =  az;
    
    return jointData;
  }//end calculateLegs
}
