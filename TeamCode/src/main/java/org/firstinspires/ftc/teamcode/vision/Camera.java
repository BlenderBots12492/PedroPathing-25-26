package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Camera {

    private static AprilTagProcessor myApriltagProcessor;
    private static AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private static VisionPortal.Builder myVisionPortalBuilder;
    private static VisionPortal myVisionPortal;
    public static MotifDetection MotifDetector;
    public DepotScan robotLocator;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            4.06, 7.5, 3.67, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS,
            0, toRadians(-75), 0, 0);

    public Camera(HardwareMap hardwareMap) { // initialize vision portal, and april tag processes
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myApriltagProcessor = (myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation).build());
        myVisionPortalBuilder.addProcessor(myApriltagProcessor);
        myVisionPortal = (myVisionPortalBuilder.build());
        MotifDetector = new MotifDetection(myVisionPortal, myApriltagProcessor);
        robotLocator = new DepotScan(myVisionPortal, myApriltagProcessor);
    }
}
