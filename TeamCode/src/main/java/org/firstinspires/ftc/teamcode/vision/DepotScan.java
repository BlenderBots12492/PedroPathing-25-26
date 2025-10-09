package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
/* INCOMPLETE DO NOT USE*/
public class DepotScan {
    private static ArrayList<AprilTagDetection> myAprilTagDetections;
    private static AprilTagDetection myAprilTagDetection;
    private static AprilTagProcessor myApriltagProcessor;
    private static AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private static VisionPortal.Builder myVisionPortalBuilder;
    private static VisionPortal myVisionPortal;
    // Describe this function...
    private void initializeVisionPortal(){
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortal = (myVisionPortalBuilder.build());
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myApriltagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myApriltagProcessor);
    }

    // Describe this function...
    public static ArrayList<AprilTagDetection> search_for_depot(){
        int[] DepotIds = {20, 24};
        myAprilTagDetections = (myApriltagProcessor.getDetections());
        ArrayList<AprilTagDetection> DetectedMotifs = new ArrayList();
        for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection2;
            if (Arrays.asList(DepotIds).contains(myAprilTagDetection.id)) {
                DetectedMotifs.add(myAprilTagDetection);
            }
        }
        return DetectedMotifs;
    }


    DepotScan() {
        initializeVisionPortal();
    }

    DepotScan(VisionPortal visionPortal, AprilTagProcessor ATagProcess) {
        myVisionPortal = visionPortal;
        myApriltagProcessor = ATagProcess;
    }
    private static Pose3D averagePose3D(Pose3D A, Pose3D B) {
        Position APos = A.getPosition();
        Position BPos = B.getPosition();
        if (APos.unit != BPos.unit) {
            BPos.toUnit(APos.unit);
        }
        Position AvgPos = new Position(APos.unit,
                (APos.x + BPos.x)/2,
                (APos.y+BPos.y)/2,
                (APos.z+BPos.z)/2,
                APos.acquisitionTime);

        return new Pose3D(AvgPos, A.getOrientation());
    }
    public static Pose3D findPos() { //Side is 'R' or 'B' red depot side or blue depot side NOT TEAM SIDES exception will be thrown otherwise.
        ArrayList<AprilTagDetection> depots = search_for_depot();
        ArrayList<Pose3D> PossiblePositions = new ArrayList<Pose3D>();
        if (depots.size() > 1) {
            return averagePose3D(depots.get(0).robotPose, depots.get(1).robotPose);
        } else {
            return depots.get(0).robotPose;
        }
    }
}
