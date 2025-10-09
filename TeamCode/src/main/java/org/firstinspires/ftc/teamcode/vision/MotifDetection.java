package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.ArrayList;

public class MotifDetection {
    private static ArrayList<AprilTagDetection> myAprilTagDetections;
    private static AprilTagDetection myAprilTagDetection;
    private static AprilTagProcessor myApriltagProcessor;
    private static AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private static VisionPortal.Builder myVisionPortalBuilder;
    private static VisionPortal myVisionPortal;
    private void initializeVisionPortal(){
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortal = (myVisionPortalBuilder.build());
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myApriltagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myApriltagProcessor);
    }

    private static ArrayList<AprilTagDetection> getVisionPortalData(){
        int[] MotifIds = {21, 22, 23};
        myAprilTagDetections = (myApriltagProcessor.getDetections());
        telemetry.update();
        ArrayList<AprilTagDetection> DetectedMotifs = new ArrayList();
        for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection2;
            if (Arrays.asList(MotifIds).contains(myAprilTagDetection.id)) {
                DetectedMotifs.add(myAprilTagDetection);
            }
        }
        return DetectedMotifs;
    }


    MotifDetection() {
        initializeVisionPortal();
    }
    MotifDetection(VisionPortal visionPortal, AprilTagProcessor ATagProcess) {
        myVisionPortal = visionPortal;
        myApriltagProcessor = ATagProcess;
    }
    public static Integer readMotif(char side) { //Side is 'R' or 'B' red depot side or blue depot side NOT TEAM SIDES exception will be thrown otherwise.
        boolean trustRight;
        if (side == 'B') {
            trustRight = true;
        } else if (side == 'R') {
            trustRight = false;
        } else {
            throw new IllegalArgumentException("Parameter for readMotif must be character R or character B for the side of the field you are on R for red depots side and B for blues depot side NOT TEAM SIDES DEPOT SIDES!!!");
        }
        ArrayList<AprilTagDetection> PossibleMotifs = getVisionPortalData();
        if (PossibleMotifs.size() > 1) {
            AprilTagDetection tag = null;
            for (AprilTagDetection motif : PossibleMotifs) {
                if (tag == null) {
                    tag = motif;
                } else if (trustRight && tag.ftcPose.y < motif.ftcPose.y) {
                    tag = motif;
                } else if (!trustRight && tag.ftcPose.y > motif.ftcPose.y) {
                    tag = motif;
                }
            }
            return tag.id;
        } else {
            return PossibleMotifs.get(0).id;
        }
    }
}
