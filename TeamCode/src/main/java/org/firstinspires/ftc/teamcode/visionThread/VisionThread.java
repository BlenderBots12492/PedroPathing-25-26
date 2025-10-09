package org.firstinspires.ftc.teamcode.visionThread;

import static java.lang.Long.signum;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Motif;

import java.lang.Thread;
public class VisionThread {
    public Camera visionStuff;
    public Pose3D position;
    public Motif motif;
    public VisionThread(Camera camera) {//Initialize camera
        visionStuff = camera;
    }
    private static class GlobalVariables {
        static boolean paused = false;
        static boolean running = true;
    }
    private class AprilTagThread extends Thread {
        public void run() {
            while (GlobalVariables.running) {
                if (GlobalVariables.paused) {
                    Pose3D position = Camera.robotLocator.findPos();
                    char side;
                    if (position.getPosition().y < 0) {
                        side = 'B';
                    } else {
                        side = 'R';
                    }
                    motif = new Motif(Camera.MotifDetector.readMotif(side));
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
    public void unPause() {
        GlobalVariables.paused = true;
    }
}