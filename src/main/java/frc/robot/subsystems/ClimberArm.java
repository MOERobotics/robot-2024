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


    private final static double MIN_VOLTAGE = 0.7;

    private final static double MAX_VOLTAGE = 3.5;


    private final static double CONVERSATION_INCHES_FACTOR = 9.956;

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
        this.speed = speed;

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

    /**
     *
     * @return distance in meters,feet, ticks
     */

    // TODO get rid of distance and percent positions
    public double getPositionDistance() {
        return climberEncoder.getPosition();
    }

    public double getPositionInches() {
        return stringPot.getVoltage()* CONVERSATION_INCHES_FACTOR;
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
        return  stringPot.getVoltage() < MAX_VOLTAGE;
    }
    public boolean canGoDown(){
        return stringPot.getVoltage() > MIN_VOLTAGE;
    }

    public double getRoll(){
        return  0;
    }

    /*
    public boolean hasChainBottom(){
        return bottomHookLimitSwitch.get();
    }

    public boolean hasChainTop(){
        return topHookLimitSwitch.get();
    }

     */

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        //  SmartDashboard.putBoolean("Top Hook has" , hasChainTop());
        //  SmartDashboard.putBoolean("Bottom Hook has" , hasChainBottom());
        SmartDashboard.putBoolean("Go up?" , canGoUp());
        SmartDashboard.putBoolean("Go down?" , canGoDown());
        SmartDashboard.putNumber("String pot Voltage:", stringPot.getVoltage());
        SmartDashboard.putNumber("Speed:", speed);
        SmartDashboard.putNumber("NavX Roll", getRoll());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
