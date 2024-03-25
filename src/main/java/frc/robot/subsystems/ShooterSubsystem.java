package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    protected double shooterRPMTolerance = 0;
    protected double maxTopSpeed;
    protected double maxBotSpeed;
    protected double desiredTopSpeed;
    protected double desiredBotSpeed;
    private double shooterSpeedTop = 0;//Store desired speeds
    private double shooterSpeedBottom = 0;

    public ShooterSubsystem() {
        shooterRPMTolerance = 5;
        maxTopSpeed = 3500;
        maxBotSpeed = 3500;
        desiredTopSpeed = 0;
        desiredBotSpeed = 0;
    }

    public void periodic() {
        SmartDashboard.putNumber("shooterTopSpeed", getShooterSpeedTop());
        SmartDashboard.putNumber("shooterBotSpeed", getShooterSpeedBottom());
        SmartDashboard.putBoolean("shooterOn", getMaxShooterSpeedTop() != 0);
    }

    public void setShooterTopSpeed(double speed) {
    }

    public void setShooterBottomSpeed(double speed) {
    }

    public void setShooterSpeeds(double topSpeed, double bottomSpeed) {
        setShooterTopSpeed(topSpeed);
        setShooterBottomSpeed(bottomSpeed);
    }

    public void setMaxShooterSpeeds(double topSpeed, double bottomSpeed) {
        setMaxShooterSpeedTop(topSpeed);
        setMaxShooterSpeedBot(bottomSpeed);
    }

    public void setDesiredTopSpeed(double speed) {
        desiredTopSpeed = speed;
    }

    public void setDesiredBotSpeed(double speed) {
        desiredBotSpeed = speed;
    }

    public double getDesiredTopSpeed() {
        return desiredTopSpeed;
    }

    public double getDesiredBotSpeed() {
        return desiredBotSpeed;
    }

    public void stopShooter() {
        setShooterSpeeds(0, 0);
    }

    public double getShooterSpeedTop() {
        return 0;
    }

    public double getShooterSpeedBottom() {
        return 0;
    }

    public double getMaxShooterSpeedTop() {
        return maxTopSpeed;
    }

    public double getMaxShooterSpeedBot() {
        return maxBotSpeed;
    }

    public void setMaxShooterSpeedTop(double speed) {
        maxTopSpeed = speed;
    }

    public void setMaxShooterSpeedBot(double speed) {
        maxBotSpeed = speed;
    }

    public void setShooterRPMTolerance(double Tolerance) {
        shooterRPMTolerance = Tolerance;
        SmartDashboard.putNumber("Shooter RPM Tolerance", Tolerance);
    }

    public double getShooterRPMTolerance() {
        return shooterRPMTolerance;
    }

    public boolean shooterAtSpeed() {
        return ((Math.abs(getDesiredTopSpeed() - getShooterSpeedTop()) <= getShooterRPMTolerance()) &&
                (Math.abs(getDesiredBotSpeed() - getShooterSpeedBottom()) <= getShooterRPMTolerance()));
    }

    public static class FortissiMOEShooter extends ShooterSubsystem{
        private final CANSparkMax shooterTop;
        private final CANSparkMax shooterBottom;

        private final SparkPIDController shooterTopController;
        private final SparkPIDController shooterBottomController;

        private final RelativeEncoder shooterTopEncoder;
        private final RelativeEncoder shooterBottomEncoder;
        private double shooterSpeedTop=0;//Store desired speeds
        private double shooterSpeedBottom=0;
        private double shooterRPMTolerance=0;
        private double maxTopSpeed, maxBotSpeed;
        private double desiredTopSpeed, desiredBotSpeed;
        public FortissiMOEShooter(int shooterTopID, int shooterBottomID, double shooterP,
                                double shooterI, double shooterD, double shooterFF) {
            shooterTop = new CANSparkMax(shooterTopID, CANSparkLowLevel.MotorType.kBrushless);
            shooterBottom = new CANSparkMax(shooterBottomID, CANSparkLowLevel.MotorType.kBrushless);
            shooterTop.setInverted(true);
            shooterBottom.setInverted(true);
            shooterTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
            shooterBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
//        shooterBottom.setSmartCurrentLimit(38);
//        shooterTop.setSmartCurrentLimit(38);

            shooterTopEncoder = shooterTop.getEncoder();
            shooterTopController = shooterTop.getPIDController();
            shooterTopController.setP(shooterP); shooterTopController.setI(shooterI);
            shooterTopController.setD(shooterD); shooterTopController.setFF(shooterFF);
            shooterTopController.setOutputRange(0, 1);

            shooterBottomEncoder = shooterBottom.getEncoder();
            shooterBottomController = shooterBottom.getPIDController();
            shooterBottomController.setP(shooterP); shooterBottomController.setI(shooterI);
            shooterBottomController.setD(shooterD); shooterBottomController.setFF(shooterFF);
            shooterTopController.setOutputRange(0, 1);
            shooterRPMTolerance=5;
            maxBotSpeed = 3500; maxTopSpeed = 3500;
            desiredBotSpeed = 0; desiredTopSpeed = 0;
        }
        @Override
        public void periodic(){
            SmartDashboard.putNumber("shooterTopSpeed", getShooterSpeedTop());
            SmartDashboard.putNumber("shooterBotSpeed", getShooterSpeedBottom());
            SmartDashboard.putBoolean("shooterOn", getMaxShooterSpeedTop() != 0);
        }

        @Override
        public void setShooterTopSpeed(double speed){

            setDesiredTopSpeed(speed);
            shooterSpeedTop=speed;
            SmartDashboard.putNumber("shooterTopDesired", speed);
            shooterTopController.setReference(speed, CANSparkBase.ControlType.kVelocity);
            //shooterTop.set(speed);
        }

        @Override
        public void setShooterBottomSpeed(double speed){

            setDesiredBotSpeed(speed);
            shooterSpeedBottom=speed;
            SmartDashboard.putNumber("shooterBottomDesired", speed);
            shooterBottomController.setReference(speed, CANSparkBase.ControlType.kVelocity);
            //shooterBottom.set(speed);
        }

        @Override
        public void setShooterSpeeds(double topSpeed, double bottomSpeed){
            setShooterTopSpeed(topSpeed); setShooterBottomSpeed(bottomSpeed);
        }
        @Override
        public void setMaxShooterSpeeds(double topSpeed, double bottomSpeed){
            setMaxShooterSpeedTop(topSpeed);
            setMaxShooterSpeedBot(bottomSpeed);
        }
        @Override
        public void setDesiredTopSpeed(double speed){
            desiredTopSpeed = speed;
        }
        @Override
        public void setDesiredBotSpeed(double speed){
            desiredBotSpeed = speed;
        }
        @Override
        public double getDesiredTopSpeed(){
            return desiredTopSpeed;
        }

        @Override
        public double getDesiredBotSpeed(){
            return desiredBotSpeed;
        }

        @Override
        public void stopShooter(){
            setShooterSpeeds(0,0);
        }

        @Override
        public double getShooterSpeedTop(){
            return shooterTopEncoder.getVelocity();
        }

        @Override
        public double getShooterSpeedBottom(){
            return shooterBottomEncoder.getVelocity();
        }
        @Override
        public double getMaxShooterSpeedTop(){
            return maxTopSpeed;
        }
        @Override
        public double getMaxShooterSpeedBot(){
            return maxBotSpeed;
        }
        @Override
        public void setMaxShooterSpeedTop(double speed){
            maxTopSpeed = speed;
        }
        @Override
        public void setMaxShooterSpeedBot(double speed){
            maxBotSpeed = speed;
        }
        @Override
        public void setShooterRPMTolerance(double Tolerance){
            shooterRPMTolerance=Tolerance;
            SmartDashboard.putNumber("Shooter RPM Tolerance", Tolerance);
        }
        @Override
        public double getShooterRPMTolerance(){
            return shooterRPMTolerance;
        }
        @Override
        public boolean shooterAtSpeed(){
            return ((Math.abs(getDesiredTopSpeed() - getShooterSpeedTop()) <= getShooterRPMTolerance()) &&
                    (Math.abs(getDesiredBotSpeed() - getShooterSpeedBottom()) <= getShooterRPMTolerance()));
        }
    }
}
