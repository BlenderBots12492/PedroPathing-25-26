package org.firstinspires.ftc.teamcode.Robot;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import android.os.SystemClock;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.tools;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.visionThread.VisionThread;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.concurrent.Callable;

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
                updatePosition();
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
        SystemClock.sleep(200);
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
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public Pose scorePose; // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(42, -12, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(42, 12, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(42, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup4Pose = new Pose(-42, -12, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup5Pose = new Pose(-42, 12, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup6Pose = new Pose(-42, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static PinpointConstants localizerConstatnts = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public PathChain[] pickUpPaths;
    public void initilizePedroPathing(tools.Team team) {
        follower = Constants.createFollower(hardwareMap);

        if (team == tools.Team.RED) {
            scorePose = new Pose(-24, 18.9, Math.toRadians(45));
            pickUpPaths = new PathChain[]{
                    follower.pathBuilder().addPath(new BezierLine(scorePose, pickup1Pose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                            .addPath(new BezierLine(pickup1Pose, new Pose(pickup1Pose.getX(), pickup1Pose.getY() * 1.25, pickup1Pose.getHeading())))
                            .build(),
                    follower.pathBuilder().addPath(new BezierLine(scorePose, pickup2Pose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                            .addPath(new BezierLine(pickup2Pose, new Pose(pickup2Pose.getX(), pickup2Pose.getY() * 1.25, pickup2Pose.getHeading())))
                            .build(),
                    follower.pathBuilder().addPath(new BezierLine(scorePose, pickup3Pose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                            .addPath(new BezierLine(pickup3Pose, new Pose(pickup3Pose.getX(), pickup3Pose.getY() * 1.25, pickup3Pose.getHeading())))
                            .build()
            };
        } else {
            scorePose = new Pose(-24, -18.9, Math.toRadians(135));
            pickUpPaths = new PathChain[]{
                    follower.pathBuilder().addPath(new BezierLine(scorePose, pickup4Pose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Pose.getHeading())
                            .addPath(new BezierLine(pickup4Pose, new Pose(pickup4Pose.getX(), pickup4Pose.getY() * 1.25, pickup4Pose.getHeading())))
                            .build(),
                    follower.pathBuilder().addPath(new BezierLine(scorePose, pickup5Pose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup5Pose.getHeading())
                            .addPath(new BezierLine(pickup5Pose, new Pose(pickup5Pose.getX(), pickup5Pose.getY() * 1.25, pickup5Pose.getHeading())))
                            .build(),
                    follower.pathBuilder().addPath(new BezierLine(scorePose, pickup6Pose))
                            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup6Pose.getHeading())
                            .addPath(new BezierLine(pickup6Pose, new Pose(pickup6Pose.getX(), pickup6Pose.getY() * 1.25, pickup6Pose.getHeading())))
                            .build()
            };
        }
    }
    public void initilizeServos() {
        launchAngleServo = hardwareMap.get(Servo.class, "launchAngleServo");
        launchBlocker = hardwareMap.get(Servo.class, "blocker");
        //launchAngleServo.setPosition(0.545); //0.2= lateral; 0.5 = higher angle
    }
    public Void updatePosition() {
        Pose3D visionPosition = vision.getLastPosition();
        if (visionPosition != visionLastPosition) {
            visionLastPosition = visionPosition;
            Pose position = new Pose(visionPosition.getPosition().x, visionPosition.getPosition().y, visionPosition.getOrientation().getYaw());
            follower.setPose(position);
        }
        return null;
    }
    public void initilizeCamera() {
        cam = new Camera(hardwareMap);
        vision = new VisionThread(cam);
        vision.start();
    }
    private Pose3D visionLastPosition;
    public Pose getPosition() {
        Pose3D visionPosition = vision.getLastPosition();
        if (visionPosition != visionLastPosition && visionPosition != null) {
            visionLastPosition = visionPosition;
            Pose position = new Pose(visionPosition.getPosition().x, visionPosition.getPosition().y, Math.toRadians(visionPosition.getOrientation().getYaw()));
            follower.setPose(position);
            return position;
        }
        return follower.getPose();
    }

    public void changeAngle(double speed) {
        launchAngleServo.setPosition(launchAngleServo.getPosition()+(speed/100));
    }
}
