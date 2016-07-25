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
    SmoothParameter body_height = new SmoothParameter(0, -0.7, 0.01);
    
    double step_height = -0.4;
    SmoothParameter step_length_x = new SmoothParameter(0, 0.2, 0.01);
    SmoothParameter step_length_y = new SmoothParameter(0, 0, 0.01);
    SmoothParameter step_rotation = new SmoothParameter(0, -0, 0.01);
    
    InertialFilter inertial = new InertialFilter(in);
    
    boolean stabilization = true;
    double lastX = 0;
    double lastY = 0;
    
    while(true)
    {
        in.update();
        inertial.update();
        
        // initialize at the first run
        if(lastServerTime == -1) {
            lastServerTime = in.getServerTime();
        }
        
        // time since the last execution
        double timeDelta = in.getServerTime() - lastServerTime;
        lastServerTime = in.getServerTime();
        
        
        // update parameters
        body_height.update();
        step_length_x.update();
        step_length_y.update();
        step_rotation.update();
        
        // 
        t += speed*timeDelta;
            
        double y = body_shift * Math.sin(t);
        double z = body_height.value() + step_height * (1.0-Math.cos(2.0*t))*0.5;

        double step_left_x = step_length_x.value() * Math.sin(t + Math.PI*0.5);
        double step_left_y = step_length_y.value() * (1+Math.sin(t + Math.PI*0.5))*0.5;
        
        double step_left_r =  -step_rotation.value() * (1+Math.sin(t + Math.PI*0.5))*0.5;
        double step_right_r =  step_rotation.value() * (1+Math.sin(t + Math.PI*1.5))*0.5;
        double step_r = (step_rotation.value() > 0)?step_left_r:step_right_r;
        
        
        // calculate pose for the left foot
        left.x = step_left_x;
        left.y = y + step_left_y;
        left.z = (y < 0)?z:body_height.value();
        left.az = step_r;
        
        if(stabilization) {
            left.x -= -inertial.getY() + (inertial.getY()-lastY)*0.2;
            left.y -= inertial.getX() - (inertial.getX()-lastX)*0.2;
            //left.z -= left.ax;
        }
        
        double step_right_x = step_length_x.value() * Math.sin(t + Math.PI*1.5);
        double step_right_y = step_length_y.value() * (1.0+Math.sin(t + Math.PI*1.5))*0.5;
        
        // calculate pose for the right foot
        right.x = step_right_x;
        right.y = y + step_right_y;
        right.z = (y > 0)?z:body_height.value();
        right.az = step_r;
        
        if(stabilization) {
            right.x -= -inertial.getY() + (inertial.getY()-lastY)*0.2;
            right.y -= inertial.getX() - (inertial.getX()-lastX)*0.2;
            //right.z += right.ax;
        }
        
        /*
        // rocking
        left.ay = Math.sin(t)*0.1;
        //left.ax = -Math.sin(t)*0.1;
        left.z = body_height.value() + left.ax;
        
        // rocking
        right.ay = Math.sin(t)*0.1;
        //right.ax = -Math.sin(t)*0.1;
        right.z = body_height.value() - right.ax;
        */
        
        // calculate the leg joint angles according to the parallel kinematic
        double[] jointData = calculateJoints(left, right);
        
        if(stabilization) {
            jointData[RobotConsts.LeftShoulderPitch] = - inertial.getY() - inertial.getX();
            jointData[RobotConsts.RightShoulderPitch] = - inertial.getY() + inertial.getX();
        }
        
        positionControl.setAllJointPositions(jointData);
        lastX = inertial.getX();
        lastY = inertial.getY();
        positionControl.update(); 
        out.sendAgentMessage();
    }
  }
  
  
  
  class InertialFilter {
      private double x;
      private double y;
      private final PerceptorInput in;
      
      InertialFilter(PerceptorInput in) {
          this.in = in;
      }
      
      public void update() {
          
          //System.out.println(in.getAcc().getX() + ", " + in.getAcc().getY() + ", " + in.getAcc().getZ());
          double ax = Math.atan2(in.getAcc().getY(), in.getAcc().getZ());
          double ay = Math.atan2(in.getAcc().getX(), in.getAcc().getZ());
          
          double gx = Math.toRadians(in.getGyro().getY())*0.02; 
          double gy = Math.toRadians(in.getGyro().getX())*0.02;
          
          double alpha = 0.02;
          
          x = (1.0-alpha)*(x + gx) + alpha*ax;
          y = (1.0-alpha)*(y + gy) + alpha*ay;
      }
      
      double getX() { return x; }
      double getY() { return y; }
  }
  
  class SmoothParameter {
      private double targetValue;
      private double currentValue;
      private final double alpha;
      
      public SmoothParameter(double initial, double target, double alpha) {
          this.currentValue = initial;
          this.targetValue = target;
          this.alpha = alpha;
      }
      
      public void update() {
          currentValue = (1.0-alpha)*currentValue + alpha*targetValue;
      }
      
      public double value() {
          return currentValue;
      }
      
      public void setTarget(double target) {
          targetValue = target;
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
