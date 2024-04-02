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

    private final AnalogInput stringPot;

    public final static double CONVERSION_FACTOR_INCHES = 9.956;

    private  double min_Inches= 0.5 * CONVERSION_FACTOR_INCHES;

    private  double max_Inches= 3.65 * CONVERSION_FACTOR_INCHES;

    private  double start_Inches = 3.65 * CONVERSION_FACTOR_INCHES;

    private final RelativeEncoder climberEncoder;

    public ClimberArm(int climberID, int stringPotID, boolean isInverted, double min_Inches, double max_Inches, double start_Inches) {


       this.min_Inches = min_Inches;
       this.max_Inches= max_Inches;
       this.start_Inches=start_Inches;
        stringPot = new AnalogInput(stringPotID);
        climberMotor = new CANSparkMax(climberID, kBrushless);
        climberMotor.setInverted(isInverted);
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
        return (stringPot.getVoltage() * CONVERSION_FACTOR_INCHES)/max_Inches ;
    }
    public boolean canGoUp(){
        return  getPositionInches()< max_Inches;
    }
    public boolean canGoDown(){
        return getPositionInches() > min_Inches;
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

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
