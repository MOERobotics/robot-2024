package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class CollectorSubsystem extends SubsystemBase {
    private final SparkMax collector;
    private final SparkMaxConfig collectorConfig;
    private final SparkClosedLoopController collectorController;
    private final DigitalInput collectorBeam;
    private boolean collectorState;

    public CollectorSubsystem(int collectorID,double collectorP, double collectorI, double collectorD, double collectorFF,  int collectorBeamID){
        this.collectorBeam = new DigitalInput(collectorBeamID);
        this.collector=new SparkMax(collectorID, SparkLowLevel.MotorType.kBrushless);
        collectorConfig = new SparkMaxConfig();
        collectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(true).smartCurrentLimit(40);
        collectorConfig.closedLoop.pid(collectorP,collectorI,collectorD).velocityFF(collectorFF).iZone(0).outputRange(-1,1);
        collector.setInverted(true);
        this.collectorController = collector.getClosedLoopController();
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

    public double getCollectorAmps(){
        return collector.getOutputCurrent();
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
