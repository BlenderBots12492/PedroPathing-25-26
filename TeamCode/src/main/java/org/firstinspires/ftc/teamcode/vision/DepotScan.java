package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.Nullable;

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

    @Nullable

    //use april tages to determin bot location
    public static AprilTagDetection search_for_depot(){
        int[] DepotIds = {20, 24};
        myAprilTagDetections = (myApriltagProcessor.getDetections());
        ArrayList<AprilTagDetection> DetectedMotifs = new ArrayList<AprilTagDetection>();
        for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {
            if (myAprilTagDetection.id == 20 || myAprilTagDetection.id == 24) {
                return myAprilTagDetection;
            }
        }
        return null;
    }


    DepotScan() {
        initializeVisionPortal();
    }

    DepotScan(VisionPortal visionPortal, AprilTagProcessor ATagProcess) {
        myVisionPortal = visionPortal;
        myApriltagProcessor = ATagProcess;
    }
    private static Pose3D averagePose3D(Pose3D A, Pose3D B) {
        //define bot's positions
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

        //send out the data we found to the main section
        return new Pose3D(AvgPos, A.getOrientation());
    }
    public Pose3D findPos(Pose3D position) {
        AprilTagDetection depot = search_for_depot();
        //if we found the robot pos then using it
        if (depot != null) {
            return depot.robotPose;
        } else {
            return position;
        }
    }
}
