package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

import java.nio.ByteBuffer;

public class ObjectDetection {
    public static final ObjectDetectionStruct struct = new ObjectDetectionStruct();

    public String classification;
    public double confidence;
    public Pose3d objectPose;

    public ObjectDetection(String classification, double confidence, Pose3d objectPose) {
        this.classification = classification;
        this.confidence = confidence;
        this.objectPose = objectPose;
    }

    public String getClassification() {
        return classification;
    }

    public double getConfidence() {
        return confidence;
    }

    public Pose3d getObjectPose() {
        return objectPose;
    }

    public static class ObjectDetectionStruct implements Struct<ObjectDetection> {

        @Override
        public Class<ObjectDetection> getTypeClass() {
            return ObjectDetection.class;
        }

        @Override
        public String getTypeString() {
            return "struct:ObjectDetection";
        }

        @Override
        public int getSize() {
            return 12 + Pose3d.struct.getSize();
        }

        @Override
        public String getSchema() {
            return "int classsification;double confidence;Pose3d objectPose";
        }

        @Override
        public ObjectDetection unpack(ByteBuffer bb) {
            int classification = bb.getInt();
            double confidence = bb.getDouble();
            Pose3d pose = Pose3d.struct.unpack(bb);
            return new ObjectDetection("note", confidence,pose);
        }

        @Override
        public void pack(ByteBuffer bb, ObjectDetection value) {
            bb.putInt(12);
            bb.putDouble(value.confidence);
            Pose3d.struct.pack(bb,value.getObjectPose());
        }
    }
}
