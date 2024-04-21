package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class ClimberArm extends SubsystemBase {

//right voltage upper limit 2.8; upper left .49; lower left 2.214; lower rigyht 2.81
    private final CANSparkMax climberMotor;
    private double speed;

    // need to change max height

    private final double climberMaxHeight = 800;

    private final SparkAnalogSensor stringPot;

    public final static double CONVERSION_FACTOR_INCHES = 9.956;

    private  double min_Inches= 0.5 * CONVERSION_FACTOR_INCHES;

    private  double max_Inches= 3.65 * CONVERSION_FACTOR_INCHES;

    private  double start_Inches = 3.65 * CONVERSION_FACTOR_INCHES;

    private final RelativeEncoder climberEncoder;

    private final SparkLimitSwitch upperLimit, lowerLimit;

    public ClimberArm(int climberID, int stringPotID, boolean isInverted, double min_Inches, double max_Inches, double start_Inches) {


       this.min_Inches = min_Inches;
       this.max_Inches= max_Inches;
       this.start_Inches=start_Inches;
        climberMotor = new CANSparkMax(climberID, kBrushless);
        climberMotor.setInverted(isInverted);
        stringPot = climberMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        upperLimit = climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        lowerLimit = climberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

        climberEncoder = climberMotor.getEncoder();
    }


    public void drive(double speed) {

/*
        if (speed > 0 && !canGoUp()) {
            speed = 0;
        }
        else if (speed < 0 && !canGoDown()) {
            speed = 0;
        }
*/

        climberMotor.set(speed);
        this.speed = speed;


    }




    public void stop(){
        drive(0);
    }



    public double getPositionVoltage(){
        return stringPot.getVoltage();
    }


    public double getPositionInches() {
        return getPositionVoltage()* CONVERSION_FACTOR_INCHES;
    }



    public double getPositionPercent() {
        return (stringPot.getVoltage() * CONVERSION_FACTOR_INCHES)/max_Inches ;
    }
    public boolean canGoUp(){
        //return  getPositionInches()< max_Inches;
        return upperLimit.isPressed();
    }
    public boolean canGoDown(){

        //return getPositionInches() > min_Inches;
        return lowerLimit.isPressed();
    }


    public double getSpeed(){
        return  speed;
    }

    public void setSpeed(double speed){
        this.speed=  speed;
    }

    public void clearStickyFault(){
        climberMotor.clearFaults();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
