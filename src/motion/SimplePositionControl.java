/*
 *
 */
package motion;

import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import util.RobotConsts;

/**
 *
 * @author Heinrich Mellmann
 */
public class SimplePositionControl {

    private final double[] targetPosition = new double[RobotConsts.JointsCount];
    private final double[] lastSpeedCommand = new double[RobotConsts.JointsCount];
    private double lastTimeOfUpdate = 0;

    private final PerceptorInput in;
    private final EffectorOutput out;
    
    public SimplePositionControl(PerceptorInput in, EffectorOutput out) {
        this.in = in;
        this.out = out;
    }

    public void setJointPosition(int jointIndex, double targetAngle) {
        if (jointIndex < 0 || jointIndex >= RobotConsts.JointsCount) {
            throw new IllegalArgumentException("jointIndex is out of range: " + jointIndex);
        }
        
        targetPosition[jointIndex] = targetAngle;
    }

    public void setAllJointPositions(double[] targetPositions) {
        if (targetPositions.length != RobotConsts.JointsCount) {
            throw new IllegalArgumentException("Expected an array of length RobotConsts.JointsCount");
        }

        for (int i = 0; i < RobotConsts.JointsCount; i++) {
            setJointPosition(i, targetPositions[i]);
        }
    }
    
    public void update() 
    {
        double timeDelta = in.getServerTime() - lastTimeOfUpdate;

        // assume default time step
        if(timeDelta <= 0) {
            timeDelta = 20.0 / 1000.0;
        }
        
        for (int i = 0; i < targetPosition.length; i++) {
            // predict the motion of the joint since the last command was sent
            double predictedJointAngle = in.getJoint(i) + lastSpeedCommand[i]*timeDelta;
            
            // calculate the new speed command
            lastSpeedCommand[i] = (targetPosition[i] - predictedJointAngle) / timeDelta;
            out.setJointCommand(i, lastSpeedCommand[i]);
        }
        
        lastTimeOfUpdate = in.getServerTime();
    }
}
