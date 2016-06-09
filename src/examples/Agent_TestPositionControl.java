/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Monika Domanska
* @version 1.1
*******************************************************************************/

package examples;

import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import motion.SimplePositionControl;
import util.Logger;
import util.RobotConsts;


public class Agent_TestPositionControl 
{

  public static void main(String args[]) {
    
    Agent_TestPositionControl agent = new Agent_TestPositionControl();
    agent.run();
    
    agent.log.printLog();
    System.out.println("Agent stopped.");
  }
  
  private final Logger log;
  private final ServerCommunication sc;
  private final PerceptorInput in;
  private final EffectorOutput out;
  private final SimplePositionControl positionControl;
  
  public Agent_TestPositionControl() 
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
    while(true) 
    {
        in.update();
        
        // move the head yaw from -45° to +45°
        double t = in.getServerTime();
        double p = Math.toRadians( Math.sin(t)*45.0 );
        positionControl.setJointPosition(RobotConsts.NeckYaw, p);
        
        // ACHTUNG: this important (!)
        positionControl.update(); 
        out.sendAgentMessage();
    }
  }
  
}
