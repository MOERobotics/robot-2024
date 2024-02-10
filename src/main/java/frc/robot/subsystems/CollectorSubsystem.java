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
        SmartDashboard.putBoolean("Beambreak",collectorBeam.get());
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
    public Command runCollectorCommandsForTeleop (final double speed, Supplier<Boolean> button1, Supplier<Boolean> button2,
                                                  Supplier<Boolean> index){
        SmartDashboard.putBoolean("started collector", collectorState);
        return Commands.run(() -> {
            double finalSpeed = speed;
            if (index.get()){
                finalSpeed = speed;
            } else if(button1.get() && !button2.get()){ //spit out
                finalSpeed = -finalSpeed;
            } else if(!button1.get()&&!button2.get()){ //nothing
                finalSpeed = 0;
            } else if(button2.get()&&isCollected()){ //beam break
                finalSpeed = 0;
            } else if(button1.get()&&button2.get()){ //both pressed?
                finalSpeed = 0;
            }
            updateCollectorSpeed(finalSpeed);
        });
    }

    public Command runCollectorForAuto(final double speed) {
        Command cmd = Commands.run(() -> updateCollectorSpeed(speed));
        if (speed > 0) {
            cmd = cmd.until(this::isCollected);
        }
        return cmd;
    }

    private void updateCollectorSpeed(double speed){
        this.setCollectorSpeed(speed);
    }
    public void stopCollector(){
        collector.set(0);
        collectorState = false;
    }


    public boolean getCollectorState (){
        return collectorState;
    }


}
