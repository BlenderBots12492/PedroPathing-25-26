package org.firstinspires.ftc.teamcode.Robot;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.VisionThread;

public class Robot {
    private class checkLaunchStatusThread extends Thread {
        public void run() {
            while (launchState != launchStatus.READY || launchState != launchStatus.IDLE) {
                if (launchWheelLeft.getVelocity() >= TARGETVELOCITY-VELOCITYTOLORANCE || launchWheelRight.getVelocity() >= TARGETVELOCITY-VELOCITYTOLORANCE) {
                    launchState = launchStatus.READY;
                }
            }
        }
    }
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor intake;
    public DcMotorEx launchWheelLeft;
    public DcMotorEx launchWheelRight;
    public VisionThread vision;
    public Camera cam;
    public int TARGETVELOCITY = 1025;
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
        launchWheelLeft.setVelocity(TARGETVELOCITY);
        checkLaunchStatusThread thread = new checkLaunchStatusThread();
        thread.start();
    }
    public boolean launch() {
        if (launchState != launchStatus.READY) {
            return false;
        }
        launchState = launchStatus.LAUNCHING;
        //LAUNCH CODE
        sleep(100);
        checkLaunchStatusThread thread = new checkLaunchStatusThread();
        thread.start();
        return true;
    }
    public void stopWheels() {
        launchWheelLeft.setVelocity(0);
        launchWheelLeft.setVelocity(0);
        launchState = launchStatus.IDLE;
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
    public Robot() {
        initilizeCamera();
        initilizeMotors();
        initilizeServos();
    }
    public void initilizeMotors() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        launchWheelLeft = hardwareMap.get(DcMotorEx.class, "launchWheelLeft");
        launchWheelRight = hardwareMap.get(DcMotorEx.class, "launchWheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launchWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchWheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchState = launchStatus.IDLE;
    }
    public void initilizeServos() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    }
    public void initilizeCamera() {
        cam = new Camera();
        vision = new VisionThread(cam);
    }
}
