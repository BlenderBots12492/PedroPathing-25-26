package org.firstinspires.ftc.teamcode.visionThread;

import static java.lang.Long.signum;

import android.provider.Settings;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Motif;

import java.lang.Thread;
import java.util.concurrent.Callable;

public class VisionThread {
    public Pose3D position;
    public Motif motif;
    public VisionThread(Camera camera) {//Initialize camera
        GlobalVariables.cam = camera;
    }
    private static class GlobalVariables {
        static boolean paused = false;
        static boolean running = true;
        static Pose3D pos;
        static Camera cam;
    }
    private class AprilTagThread extends Thread {
        public void run() {
            YawPitchRollAngles orient = new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);
            Position pos = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
            Pose3D position = new Pose3D(pos, orient);
            while (GlobalVariables.running) {
                if (!GlobalVariables.paused) {
                    position = GlobalVariables.cam.robotLocator.findPos(position);
                    GlobalVariables.pos = position;
                    /*char side;
                    if (position.getPosition().y < 0) {
                        side = 'B';
                    } else {
                        side = 'R';
                    }
                    motif = new Motif(Camera.MotifDetector.readMotif(side));*/
                }
            }
        }
    }
    public void start() {
        GlobalVariables.running = true;
        AprilTagThread thread = new AprilTagThread();
        thread.start();
    }
    public void pause() {
        GlobalVariables.paused = !GlobalVariables.paused;
    }
    public void stop() {
        GlobalVariables.running = false;
    }
    public Pose3D getLastPosition() {
        return GlobalVariables.pos;
    }
    public void unPause() {
        GlobalVariables.paused = true;
    }
}