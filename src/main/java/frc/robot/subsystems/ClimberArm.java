package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class ClimberArm extends SubsystemBase {


    private final CANSparkMax climberMotorRight;


    private final CANSparkMax climberMotorLeft;


    private double speed;

    // need to change max height

    private final double climberMaxHeight = 800;

    private boolean rightArmLimit;
    private boolean LeftArmLimit;

    private double tolerance;



    private DigitalInput topHookLimitSwitch;

    private DigitalInput bottomHookLimitSwitch;

    private final RelativeEncoder climberEncoder;

    private Pigeon2 pigeon2 = new Pigeon2(88);

   private AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);


    public ClimberArm(int climberIDRight,int climberIDLeft, int topHookLimitSwitchID, int bottomHookLimitSwitchID) {


        topHookLimitSwitch= new DigitalInput(topHookLimitSwitchID);
        bottomHookLimitSwitch= new DigitalInput(bottomHookLimitSwitchID);
        climberMotorRight = new CANSparkMax(climberIDRight, kBrushless);
        climberMotorLeft = new CANSparkMax(climberIDLeft, kBrushless);
        climberEncoder = climberMotorRight.getEncoder();


    }


    public void driveRight(double speed){
        climberMotorRight.set(speed);
    }
    public void driveLeft(double speed){
        climberMotorLeft.set(speed);
    }


    public void stop(){
        climberMotorRight.stopMotor();
    }

    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }


    public double getPositionDistance() {
        return climberEncoder.getPosition();
    }


    public double getPositionPercent() {
        return climberEncoder.getPosition()/climberMaxHeight ;
    }



    public boolean canGoUpRight(){
       return   !climberMotorRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }


    public boolean canGoUpLeft(){
        return   !climberMotorRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }


    public boolean canGoDownRight(){
        return   !climberMotorLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }


    public boolean canGoDownLeft(){
        return   !climberMotorLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public double getRoll(){

      return  navx.getRoll();

    }



    public void pullUp () {
        double roll = navx.getRoll();
        double finalSpeed;

        if (canGoUpRight() && canGoUpLeft()) {
            driveLeft(speed);
            driveRight(speed);
        } else if (roll > tolerance) {
            climberMotorLeft.stopMotor();
            if (canGoDownRight()) {
                finalSpeed = -speed;
                climberMotorRight.set(finalSpeed);
            } else {
                climberMotorRight.stopMotor();
            }
        } else if (roll < -tolerance) {
            climberMotorRight.stopMotor();
            if (canGoDownLeft()) {
                finalSpeed = -speed;
                climberMotorLeft.set(finalSpeed);
            } else {
                climberMotorLeft.stopMotor();
            }
        } else {
            climberMotorLeft.stopMotor();
            climberMotorRight.stopMotor();
        }
    }



    public boolean hasChainBottom(){
        return bottomHookLimitSwitch.get();
    }

    public boolean hasChainTop(){
        return topHookLimitSwitch.get();
    }


    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler
        SmartDashboard.putBoolean("Top Hook has" , hasChainTop());
        SmartDashboard.putBoolean("Bottom Hook has" , hasChainBottom());
        SmartDashboard.putBoolean("Go up?" , canGoUpRight());
        SmartDashboard.putBoolean("Go down?" , canGoDownRight());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
