package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;

public interface Gyroscope {

    double getYaw();

    double getPitch();

    double getRoll();

    void setYaw(double yaw);

    default void reset() {
        setYaw(0);
    }

    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(getYaw(),-180,180));
    }

    public static class PigeonGyro implements Gyroscope {
        public WPI_Pigeon2 pigeon;
        public PigeonGyro(WPI_Pigeon2 pigeon2){
            this.pigeon=pigeon2;
        }
        public PigeonGyro(int pigeonId) {
            this(new WPI_Pigeon2(pigeonId));
        }
        @Override
        public double getYaw() {
            return pigeon.getYaw();
        }

        @Override
        public void setYaw(double yaw) {
            pigeon.setYaw(yaw);
        }
        @Override
        public double getPitch( ) {
           return pigeon.getPitch();
        }
        @Override
        public double getRoll() {
            return pigeon.getRoll();
        }
    }

    public static class NavXGyro implements Gyroscope {
        public AHRS navx;
        private double offset = 0;
        public NavXGyro(AHRS navx){
            this.navx=navx;
        }
        public NavXGyro() {
            this(new AHRS(I2C.Port.kMXP,(byte) 50));
        }
        @Override
        public double getYaw() {
            return navx.getYaw() + offset;
        }

        @Override
        public void setYaw(double yaw) {
            navx.reset();
            offset = yaw;
        }

        @Override
        public double getPitch( ) {
            return navx.getPitch();
        }

        @Override
        public double getRoll( ) {
            return navx.getRoll();
        }
    }
}
