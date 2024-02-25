package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AnalogInput;
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


    private final CANSparkMax climberMotor;




    private double speed;

    // need to change max height

    private final double climberMaxHeight = 800;

    private boolean ArmLimit;

    private double tolerance;



    private DigitalInput topHookLimitSwitch;

    private DigitalInput bottomHookLimitSwitch;

    private final RelativeEncoder climberEncoder;

    // TODO make analog input and new canGoUp and down methods(0.2 -1.6)



   private final AHRS navx;


    public ClimberArm(int climberID, int topHookLimitSwitchID, int bottomHookLimitSwitchID, AHRS navx) {


        this.navx = navx;

        topHookLimitSwitch= new DigitalInput(topHookLimitSwitchID);
        bottomHookLimitSwitch= new DigitalInput(bottomHookLimitSwitchID);
        climberMotor = new CANSparkMax(climberID, kBrushless);
        climberEncoder = climberMotor.getEncoder();
    }


    public void drive(double speed) {
        if (speed > 0 && canGoUp()) {
            climberMotor.set(speed);
        }
        else if (speed < 0 && canGoDown()) {
            climberMotor.set(speed);
        } else {
            climberMotor.stopMotor();
        }
    }


    public void stop(){
        climberMotor.stopMotor();
    }

    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     *
     * @return distance in meters,feet, ticks
     */
    public double getPositionDistance() {
        return climberEncoder.getPosition();
    }


    public double getPositionPercent() {
        return climberEncoder.getPosition()/climberMaxHeight ;
    }



    public boolean canGoUp(){
       return   !climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }




    public boolean canGoDown(){
        return   !climberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public double getRoll(){

      return  navx.getRoll();

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
        SmartDashboard.putBoolean("Go up?" , canGoUp());
        SmartDashboard.putBoolean("Go down?" , canGoDown());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
