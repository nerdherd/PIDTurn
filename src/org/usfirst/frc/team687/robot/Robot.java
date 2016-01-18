
package org.usfirst.frc.team687.robot;

import com.kauailabs.navx_mxp.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	VictorSP ftLeft;
	VictorSP ftRight;
	VictorSP bkLeft;
	VictorSP bkRight;
	Joystick joy;
	
	double kP = 0;
	double kI = 0;
	double kD = 0;
	
	double error = 0;
	double lastError = 0;
	double integration = 0;
	
	double tolerance = 2;
	
	AHRS nav;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	ftLeft = new VictorSP(0);
    	ftRight = new VictorSP(3);
    	bkLeft = new VictorSP(1);
    	bkRight = new VictorSP(4);
    	
    	joy = new Joystick(0);
    	
    	nav = new AHRS(new SerialPort(57600, SerialPort.Port.kMXP));
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
        if(joy.getRawButton(7))	{
        	kP -= 0.0001;
        }	else if(joy.getRawButton(8))	{
        	kP += 0.0001;
        }	else if(joy.getRawButton(9))	{
        	kI -= 0.000001;
        }	else if(joy.getRawButton(10))	{
        	kI += 0.000001;
        }	else if(joy.getRawButton(11))	{
        	kD -= 0.001;
        }	else if(joy.getRawButton(12))	{
        	kD += 0.001;
        }	else if(joy.getRawButton(5))	{
        	tolerance +=1;
        }	else if(joy.getRawButton(3))	{
        	tolerance -=1;
        }
        
        double pow = 0;
        
        lastError = error;
        double yaw = (nav.getYaw()+360)%360;
        double angle = Math.sqrt(joy.getY()*joy.getY() + joy.getX()*joy.getX()) > 0.2 ? (Math.atan2(joy.getY(), joy.getX())*180/Math.PI + 450) % 360 : -9001;
        
        error = angle != -9001 ? shortestRotation(angle, yaw): 0;
        
        double p = kP * error;
        integration += 0.02*(error+lastError)/2;
        double i = kI * integration;
        double d = kD * (error - lastError);
        
        if(Math.abs(error) > tolerance)	{
	        pow = p+i-d;
        }	else	{
        	pow = 0;
        	integration = 0;
        }
        
        SmartDashboard.putNumber("P", kP);
        SmartDashboard.putNumber("I", kI);
        SmartDashboard.putNumber("D", kD);
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Desired", angle);
        SmartDashboard.putNumber("pow", pow);

        ftLeft.set(pow);
        ftRight.set(pow);
        bkLeft.set(pow);
        bkRight.set(pow);
    }
    
    private double shortestRotation(double desired, double heading)  {
        double e = heading - desired;
        if(e > 180)    {
            e = -(Math.abs(360 - e)%180);
        }   else if(e < -180)   {
            e = Math.abs(360 + e)%180;
        }
        
        return e;
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
