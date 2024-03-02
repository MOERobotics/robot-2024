package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class CollectorSubsystem extends SubsystemBase {
    private final CANSparkMax collector;
    private final SparkPIDController collectorController;
    private final DigitalInput collectorBeam;
    private boolean collectorState;

    public CollectorSubsystem(int collectorID,double collectorP, double collectorI, double collectorD, double collectorFF,  int collectorBeamID){
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
    public boolean isCollected(){
        SmartDashboard.putBoolean("Beambreak", collectorBeam.get());
        return collectorBeam.get();
    }
    public void setCollectorSpeed(double speed){
        collector.set(speed);

        if(speed==0){
            collectorState = false;
        } else {
            collectorState = true;
        }
    }


    public Command runCollectorForAuto(final double speed) {
        Command cmd = Commands.run(() -> updateCollectorSpeed(speed));
        if (speed > 0) {
            cmd = cmd.until(this::isCollected);
        }
        return cmd;
    }

    public void updateCollectorSpeed(double speed){
        this.setCollectorSpeed(speed);
    }
    public void stopCollector(){
        collector.set(0);
        collectorState = false;
    }

    @Override
    public void periodic(){
        isCollected();
    }


    public boolean getCollectorState (){
        return collectorState;
    }


}
