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
public class ClimberArmSubsystem extends SubsystemBase {
    public final static double CONVERSION_FACTOR_INCHES = 9.956;
    private final double climberMaxHeight = 800;
    protected double min_Inches = 0.5 * CONVERSION_FACTOR_INCHES;
    protected double max_Inches = 3.65 * CONVERSION_FACTOR_INCHES;
    protected double start_Inches = 3.65 * CONVERSION_FACTOR_INCHES;
    private double speed;

    public ClimberArmSubsystem(double min_Inches, double max_Inches, double start_Inches) {
        this.min_Inches = min_Inches;
        this.max_Inches = max_Inches;
        this.start_Inches = start_Inches;
    }

    public void drive(double speed) {}

    public void stop() {
        drive(0);
    }

    public double getPositionVoltage() {
        return 0;
    }

    public double getPositionInches() {
        return getPositionVoltage() * CONVERSION_FACTOR_INCHES;
    }

    public double getPositionPercent() {
        return 0;
    }

    public boolean canGoUp() {
        return getPositionInches() < max_Inches;
    }

    public boolean canGoDown() {
        return getPositionInches() > min_Inches;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public static class FortissiMOEClimberArm extends ClimberArmSubsystem{
        private final CANSparkMax climberMotor;
        private double speed;
        private final double climberMaxHeight = 800;
        private final AnalogInput stringPot;
        public final static double CONVERSION_FACTOR_INCHES = 9.956;
        private double min_Inches= 0.5 * CONVERSION_FACTOR_INCHES;
        private double max_Inches= 3.65 * CONVERSION_FACTOR_INCHES;
        private double start_Inches = 3.65 * CONVERSION_FACTOR_INCHES;
        private final RelativeEncoder climberEncoder;
        public FortissiMOEClimberArm(int climberID, int stringPotID, boolean isInverted, double min_Inches, double max_Inches, double start_Inches) {
            super(min_Inches, max_Inches, start_Inches);
            stringPot = new AnalogInput(stringPotID);
            climberMotor = new CANSparkMax(climberID, kBrushless);
            climberMotor.setInverted(isInverted);
            climberEncoder = climberMotor.getEncoder();
        }

        @Override
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
        @Override
        public void stop(){
            drive(0);
        }
        @Override
        public double getPositionVoltage(){
            return stringPot.getVoltage();
        }

        @Override
        public double getPositionInches() {
            return getPositionVoltage()* CONVERSION_FACTOR_INCHES;
        }
        @Override
        public double getPositionPercent() {
            return (stringPot.getVoltage() * CONVERSION_FACTOR_INCHES)/max_Inches ;
        }
        @Override
        public boolean canGoUp(){
            return  getPositionInches()< max_Inches;
        }
        @Override
        public boolean canGoDown(){
            return getPositionInches() > min_Inches;
        }

        @Override
        public double getSpeed(){
            return  speed;
        }
        @Override
        public void setSpeed(double speed){
            this.speed=  speed;
        }

    }

}
