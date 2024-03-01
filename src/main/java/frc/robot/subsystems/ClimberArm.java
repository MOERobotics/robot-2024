package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
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



    // private final DigitalInput topHookLimitSwitch;

    // private final DigitalInput bottomHookLimitSwitch;


    private final AnalogInput stringPot;

    private final static double CONVERSION_FACTOR_INCHES = 9.956;

    private final static double MIN_INCHES = 0.5 * CONVERSION_FACTOR_INCHES;

    private final static double MAX_INCHES = 3.6 * CONVERSION_FACTOR_INCHES;


    private final RelativeEncoder climberEncoder;

    // TODO make analog input and new canGoUp and down methods(0.2 -1.6)





    public ClimberArm(int climberID, /* int topHookLimitSwitchID, int bottomHookLimitSwitchID, */ int stringPotID) {
        stringPot = new AnalogInput(stringPotID);
        // topHookLimitSwitch= new DigitalInput(topHookLimitSwitchID);
        // bottomHookLimitSwitch= new DigitalInput(bottomHookLimitSwitchID);
        climberMotor = new CANSparkMax(climberID, kBrushless);
        climberEncoder = climberMotor.getEncoder();
    }


    public void drive(double speed) {

        if (speed > 0 && !canGoUp()) {
            speed = 0;
        }
        else if (speed < 0 && !canGoDown()) {
            speed = 0;
        }

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
        return climberEncoder.getPosition()/climberMaxHeight ;
    }

    /*
    public double getPositionPercent() {
        return stringPot.getVoltage()/MAX_VOLTAGE ;
    }
     */



    public boolean canGoUp(){
        return  getPositionInches()< MAX_INCHES;
    }
    public boolean canGoDown(){
        return getPositionInches() > MIN_INCHES;
    }


    public double getSpeed(){
        return  speed;
    }

    public void setSpeed(double speed){
        this.speed=  speed;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        //  SmartDashboard.putBoolean("Top Hook has" , hasChainTop());
        //  SmartDashboard.putBoolean("Bottom Hook has" , hasChainBottom());

      //  SmartDashboard.putNumber("NavX Roll", getRoll());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
