package org.firstinspires.ftc.teamcode.Robot;

import static android.os.SystemClock.sleep;

import static java.lang.Math.abs;

import android.os.SystemClock;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.tools;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.visionThread.VisionThread;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Robot {
    private class checkLaunchStatusThread extends Thread {
        public void run() {
            while (launchState != launchStatus.READY || launchState != launchStatus.IDLE) {
                if (abs(launchWheelLeft.getVelocity()) >= TARGETVELOCITY-VELOCITYTOLORANCE || abs(launchWheelRight.getVelocity()) >= TARGETVELOCITY-VELOCITYTOLORANCE) {
                    launchState = launchStatus.READY;
                }
            }
        }
    }
    private VoltageSensor myControlHubVoltageSensor;
    private class voltageChecker extends Thread {
        public void run() {
            double controllerMiniumumVoltage = 8;

            myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            boolean launchEnabled = true;
            while (true) {
                double presentVoltage = myControlHubVoltageSensor.getVoltage();
                if (8 >= presentVoltage) {
                    launchEnabled = false;
                    intake.setPower(0);
                } else {
                    launchEnabled = true;
                }
                if (!launchEnabled) {
                    launchWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    launchWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    stopWheels();

                } else {
                    launchWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    launchWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }
    }
    public void checkVoltage() {
        voltageChecker thread = new voltageChecker();
        thread.start();
    }
    private Gamepad launchPad;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor intake;
    public DcMotorEx launchWheelLeft;
    public DcMotorEx launchWheelRight;
    public VisionThread vision;
    public Camera cam;
    public Servo launchAngleServo;
    public Servo launchBlocker;
    public int TARGETVELOCITY = 700;
    public int VELOCITYTOLORANCE = 10;
    public enum launchStatus {
        LAUNCHING,
        IDLE,
        READY,
        SPEEDING_UP
    }
    public launchStatus launchState;
    public void startWheels() {
        launchState = launchStatus.SPEEDING_UP;
        launchWheelLeft.setVelocity(TARGETVELOCITY);
        launchWheelRight.setVelocity(TARGETVELOCITY);

        checkLaunchStatusThread thread = new checkLaunchStatusThread();
        thread.start();
    }
    public boolean launch() {
        if (launchState != launchStatus.READY) {
            return false;
        }
        launchState = launchStatus.LAUNCHING;
        launchBlocker.setPosition(0);
        sleep(200);
        block();
        checkLaunchStatusThread thread = new checkLaunchStatusThread();
        thread.start();
        return true;
    }
    public void stopWheels() {
        launchWheelLeft.setVelocity(0);
        launchWheelRight.setVelocity(0);
        launchState = launchStatus.IDLE;
    }
    public void block() {
        launchBlocker.setPosition(0.55);
    }
    public void intake() {
        if (intake.getPower() > 0.5) {
            intake.setPower(0);
        } else {
            intake.setPower(1);
        }
    }

    public void setPowers(double forward, double strafe, double turn) { //Right is positive
        double backLeftPwr = forward+strafe-turn;
        double backRightPwr = forward-strafe+turn;
        double frontLeftPwr = forward-strafe-turn;
        double frontRightPwr = forward+strafe+turn;
        backLeft.setPower(backLeftPwr);
        backRight.setPower(backRightPwr);
        frontLeft.setPower(frontLeftPwr);
        frontRight.setPower(frontRightPwr);
    }
    HardwareMap hardwareMap;
    public Robot(HardwareMap HWMap) {
        hardwareMap = HWMap;
        //initilizeCamera();
        initilizeMotors();
        initilizeServos();
    }
    public void initilizeMotors() {
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchWheelLeft = hardwareMap.get(DcMotorEx.class, "launchWheelLeft");
        launchWheelRight = hardwareMap.get(DcMotorEx.class, "launchWheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launchWheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchWheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launchWheelRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        //launchWheelLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launchWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchState = launchStatus.IDLE;
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Pose startPose; // Start Pose
    private Pose scorePose; // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(42, -12, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(42, 12, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(42, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup4Pose = new Pose(-42, -12, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup5Pose = new Pose(-42, 12, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup6Pose = new Pose(-42, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    public void initilizePedroPathing(Pose position, tools.Team team) {
        startPose = position;
        if (team == tools.Team.RED) {
            scorePose = new Pose(-3, 9.5, Math.toRadians(-45));
        } else {
            scorePose = new Pose(-3, -9.5, Math.toRadians(-135));
        }

    }
    public void goTo() {

    }
    public void initilizeServos() {
        launchAngleServo = hardwareMap.get(Servo.class, "launchAngleServo");
        launchBlocker = hardwareMap.get(Servo.class, "blocker");
        //launchAngleServo.setPosition(0.545); //0.2= lateral; 0.5 = higher angle
    }
    public void initilizeCamera() {
        cam = new Camera();
        vision = new VisionThread(cam);
    }
    public void changeAngle(double speed) {
        launchAngleServo.setPosition(launchAngleServo.getPosition()+(speed/100));
    }
}
