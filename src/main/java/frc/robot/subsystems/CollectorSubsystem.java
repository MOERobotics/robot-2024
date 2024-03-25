package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CollectorSubsystem extends SubsystemBase {

    public boolean isCollected() { return false;}
    public void setCollectorSpeed(double speed){}


    public Command runCollectorForAuto(final double speed) {
        Command cmd = Commands.none();
        return cmd;
    }

    public void updateCollectorSpeed(double speed) {
    }

    public void stopCollector() {
    }

    @Override
    public void periodic() {
        isCollected();
    }

    public boolean getCollectorState() {
        return false;
    }

    public static class FortissiMOECollector extends CollectorSubsystem{
        private final CANSparkMax collector;
        private final SparkPIDController collectorController;
        private final DigitalInput collectorBeam;
        private boolean collectorState;

        public FortissiMOECollector(int collectorID,double collectorP, double collectorI, double collectorD, double collectorFF,  int collectorBeamID){
            this.collectorBeam = new DigitalInput(collectorBeamID);
            this.collector=new CANSparkMax(collectorID, CANSparkLowLevel.MotorType.kBrushless);
            collector.setIdleMode(CANSparkBase.IdleMode.kBrake);
            collector.setInverted(true);
            this.collectorController = collector.getPIDController();
            collectorController.setP(collectorP);
            collectorController.setI(collectorI);
            collectorController.setIZone(0);
            collectorController.setD(collectorD);
            collectorController.setFF(collectorFF);
            collectorController.setOutputRange(-1, 1);
        }

        @Override
        public boolean isCollected(){
            SmartDashboard.putBoolean("Beambreak", collectorBeam.get());
            return collectorBeam.get();
        }

        @Override
        public void setCollectorSpeed(double speed){
            collector.set(speed);

            if(speed==0){
                collectorState = false;
            } else {
                collectorState = true;
            }
        }

        @Override
        public Command runCollectorForAuto(final double speed) {
            Command cmd = Commands.run(() -> updateCollectorSpeed(speed));
            if (speed > 0) {
                cmd = cmd.until(this::isCollected);
            }
            return cmd;
        }
        @Override
        public void updateCollectorSpeed(double speed){
            this.setCollectorSpeed(speed);
        }

        @Override
        public void stopCollector(){
            collector.set(0);
            collectorState = false;
        }

        @Override
        public void periodic(){
            isCollected();
        }

        @Override
        public boolean getCollectorState (){
            return collectorState;
        }
    }
}
