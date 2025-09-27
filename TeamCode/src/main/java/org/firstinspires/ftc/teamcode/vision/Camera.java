package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Camera {

    private static ArrayList<AprilTagDetection> myAprilTagDetections;
    private static AprilTagDetection myAprilTagDetection;
    private static AprilTagProcessor myApriltagProcessor;
    private static AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private static VisionPortal.Builder myVisionPortalBuilder;
    private static VisionPortal myVisionPortal;
    public static MotifDetection MotifDetector;
    public static DepotScan robotLocator;

    Camera() {
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortal = (myVisionPortalBuilder.build());
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam 1"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myApriltagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myApriltagProcessor);
        MotifDetector = new MotifDetection(myVisionPortal, myApriltagProcessor);
        robotLocator = new DepotScan(myVisionPortal, myApriltagProcessor);
    }
}
