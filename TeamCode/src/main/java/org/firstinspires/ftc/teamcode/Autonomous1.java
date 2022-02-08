package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import CupertinoRobotics.support.Hardware.Gyro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.*;
import com.qualcomm.robotcore.hardware.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AutonomousRedLeft")
public class Autonomous1 extends LinearOpMode {

    //hardware components
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private CRServo carouselMotor = null;
    private Gyro gyro = null;
    private BNO055IMU imu;

    //subject to change after rigorous testing
    final double tpi = 46;
    final int MAX_ARM_POSITION = -1350;
    final int MIDDLE_ARM_POSITION = -800;
    final int BOTTOM_ARM_POSITION = -300;
    final int MIN_ARM_POSITION = 0;
    final double tpd = (MIN_ARM_POSITION - MAX_ARM_POSITION) / 120.0;
    double power = 0.25;
    final double armPower = 0.45;

    private Servo clawServo = null;
    // Carousel servo
    private CRServo cServo = null;

    // Position/encoder variables
    double PID = -0.04;
    boolean armButtonPressed;


    //states
    boolean forward = false;
    boolean right = false;
    boolean ArmUp = false;

    //helper variables
    List<Double> leftencodervalues = new ArrayList<Double>();
    List<Double> rightencodervalues = new ArrayList<Double>();
    List<Double> armencodervalues = new ArrayList<Double>();
    List<Double> Gyrovalues = new ArrayList<Double>();
    List<Double> servotimevalues = new ArrayList<Double>();
    double rotcount = 0;
    boolean angleneg = false;

    OpenCvCamera cam;
    int locationDuck = 0;

    @Override
    public void runOpMode() {
        //hardware initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        carouselMotor = hardwareMap.get(CRServo.class, "c_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        gyro = new Gyro(hardwareMap, AngleUnit.DEGREES);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // open cv stuff
        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // handle if cam return null?

        DuckScanner scanner = new DuckScanner(telemetry);
        cam.setPipeline(scanner);

//        cam.openCameraDeviceAsync(
//                () -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
//        );

        // setting up the camera device and streaming system
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //please die
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Scanning");
            telemetry.update();

            while (locationDuck == 0){
                locationDuck = scanner.duckLocation();
                sleep(500);
            }

            // duck location is the variable that says where the duck was
            // use that to determine where to place our shipping element

            telemetry.addData("Duck Location", scanner.duckLocation());
            locationDuck = scanner.duckLocation();
            telemetry.update();

            //this code moves robot from starting to in front of the shipping hub and drops freight on the middle level
            clawServo.setPosition(0);
            sleep(1500);
            if (locationDuck == 1) { // left location - bottom level
                if (true) {
                    armMotor.setTargetPosition(BOTTOM_ARM_POSITION);
                    //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (armMotor.getCurrentPosition() > BOTTOM_ARM_POSITION) {
                        armMotor.setPower(-armPower);
                        telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                        telemetry.update();
                        sleep(20);
                    }
                    armMotor.setPower(PID);
                }
            } else if (locationDuck == 3) { // right spot - top level
                if (true) {
                    armMotor.setTargetPosition(MAX_ARM_POSITION);
                    //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (armMotor.getCurrentPosition() > MAX_ARM_POSITION) {
                        armMotor.setPower(-armPower);
                        telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                        telemetry.update();
                        sleep(20);
                    }
                    armMotor.setPower(PID);
                }
            } else if (locationDuck == 2) { // middle spot - middle level
                if (true) {
                    armMotor.setTargetPosition(MIDDLE_ARM_POSITION);
                    //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (armMotor.getCurrentPosition() > MIDDLE_ARM_POSITION) {
                        armMotor.setPower(-armPower);
                        telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                        telemetry.update();
                        sleep(20);
                    }
                    armMotor.setPower(PID);
                }
            }

            forward(5);
            startSleep(500);

            if (locationDuck == 1) { //bottom
                turnRight(32);
                startSleep(500);
                forward(28);
                startSleep(1500);
            } else if (locationDuck == 3) { //top
                turnRight(32);
                startSleep(500);
                forward(30);
                startSleep(1500);
            } else if (locationDuck == 2) { //middle
                turnRight(32);
                startSleep(500);
                forward(30);
                startSleep(1500);
            }




            /*
            turnRight(90);
            startSleep(1000);
            forward(22);
            startSleep(1500);
            turnLeft(100);
            startSleep(750 );
            */

            clawServo.setPosition(1);
            sleep(1000);
            backward(5);
            startSleep(500);
            /*
            if (true){
                armMotor.setTargetPosition(MIN_ARM_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (armMotor.getCurrentPosition()<=MIN_ARM_POSITION){
                    armMotor.setPower(armPower);
                    telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                    telemetry.update();
                    sleep(20);
                }
                armMotor.setPower(PID);
            }
             */

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            if (locationDuck == 1) { //bottom
                turnRight(22);
                startSleep(800);
                backward(42);
                startSleep(2000);
                carouselMotor.setPower(0.5);
                leftDrive.setPower(-0.01);
                rightDrive.setPower(-0.01);
                sleep(500);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(2500);
                carouselMotor.setPower(0);
            } else if (locationDuck == 3) { //top
                turnRight(22);
                startSleep(800);
                backward(42);
                startSleep(2000);
                carouselMotor.setPower(0.5);
                leftDrive.setPower(-0.01);
                rightDrive.setPower(-0.01);
                sleep(500);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(2500);
                carouselMotor.setPower(0);
            } else if (locationDuck == 2) { //middle
                turnRight(22);
                startSleep(800);
                backward(42);
                startSleep(2000);
                carouselMotor.setPower(0.5);
                leftDrive.setPower(-0.01);
                rightDrive.setPower(-0.01);
                sleep(500);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(2500);
                carouselMotor.setPower(0);
            }

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            forward(3);
            startSleep(1000);
            turnLeft(85);
            startSleep(1000);
            forward(26);
            startSleep(2000);
            break;
        }
    }

    //spins servo 7 seconds per duck (hardcoded in)
    public void spinCarousel(int numducks) {
        carouselMotor.setPower(0.5);
        double time = getRuntime();
        servotimevalues.add(time + 7000 * numducks);
    }

    //Helps calculate continous angle value (- infinity to + infinity by checking when the angle switches signs
    public void checkJump() {
        if (right) {
            if (angleneg) {
                if (gyro.getAngle() >= 0) {
                    rotcount -= 1;
                    angleneg = false;
                }
            }
            if (gyro.getAngle() < 0) {


                angleneg = true;
            }
        } else {
            if (!angleneg) {
                if (gyro.getAngle() < 0) {
                    rotcount += 1;
                    angleneg = true;
                }
            }
            if (gyro.getAngle() > 0) {


                angleneg = false;
            }
        }
    }

    //Method that helps asynchronous movement and motor access by checking all the motors and seeing wether they should be ended or not
    public void endMotors() {
        if (servotimevalues.size() > 0) {
            if (servotimevalues.get(0) >= getRuntime()) {
                carouselMotor.setPower(0);
                servotimevalues.remove(0);
            }
        }
        if (forward) {
            if (leftencodervalues.size() > 0) {
                if (leftDrive.getCurrentPosition() >= leftencodervalues.get(0)) {
                    leftencodervalues.remove(0);
                    leftDrive.setPower(0);

                }
            }
            if (rightencodervalues.size() > 0) {
                if (rightDrive.getCurrentPosition() >= rightencodervalues.get(0)) {
                    rightencodervalues.remove(0);
                    rightDrive.setPower(0);
                }
            }


        } else {
            if (leftencodervalues.size() > 0) {
                if (leftDrive.getCurrentPosition() <= leftencodervalues.get(0)) {
                    leftencodervalues.remove(0);
                    leftDrive.setPower(0);

                }
            }
            if (rightencodervalues.size() > 0) {
                if (rightDrive.getCurrentPosition() <= rightencodervalues.get(0)) {
                    rightencodervalues.remove(0);
                    rightDrive.setPower(0);
                }
            }


        }
        if (right) {
            if (Gyrovalues.size() > 0) {
                if (getRealAngle() <= Gyrovalues.get(0)) {
                    Gyrovalues.remove(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(0);
                } else {
                    leftDrive.setPower(power);
                    rightDrive.setPower(-power);
                }
            }
        } else {
            if (Gyrovalues.size() > 0) {
                if (getRealAngle() >= Gyrovalues.get(0)) {
                    Gyrovalues.remove(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(0);
                } else {
                    leftDrive.setPower(-power);
                    rightDrive.setPower(power);
                }
            }
        }
        if (ArmUp) {
            if (armencodervalues.size() > 0) {
                if (armMotor.getCurrentPosition() <= armencodervalues.get(0)) {
                    armMotor.setPower(0);
                    armencodervalues.remove(0);
                }
            }
        } else {
            if (armencodervalues.size() > 0) {
                if (armMotor.getCurrentPosition() >= armencodervalues.get(0)) {
                    armMotor.setPower(0);
                    armencodervalues.remove(0);
                }
            }
        }
    }

    //moves arm up given a positive parameter in degrees
    public void movingArmUp(int degrees) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmUp = true;
        if (armMotor.getCurrentPosition() - degrees * tpd < MAX_ARM_POSITION) {
            armencodervalues.add((double) MAX_ARM_POSITION);
        } else {
            armencodervalues.add(armMotor.getCurrentPosition() - degrees * tpd);
        }
        armMotor.setPower(-armPower);
    }

    //moves arm down given a positive parameter in degrees
    public void movingArmDown(int degrees) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmUp = false;
        if (armMotor.getCurrentPosition() + degrees * tpd > MIN_ARM_POSITION) {
            armencodervalues.add((double) MIN_ARM_POSITION);
        } else {
            armencodervalues.add(armMotor.getCurrentPosition() + degrees * tpd);
        }
        armMotor.setPower(armPower);
    }

    //moves robot forward given a positive parameter in inches
    //temporarily ticks
    public void forward(double inches) {
        forward = true;
        double distanceInTicks = inches * tpi;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftencodervalues.add(leftDrive.getCurrentPosition() + distanceInTicks);
        rightencodervalues.add(rightDrive.getCurrentPosition() + distanceInTicks);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    //moves robot backward given a positive parameter in inches
    public void backward(double inches) {
        forward = false;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double distanceInTicks = inches * tpi;
        leftencodervalues.add(leftDrive.getCurrentPosition() - distanceInTicks);
        rightencodervalues.add(rightDrive.getCurrentPosition() - distanceInTicks);
        leftDrive.setPower(-power);
        rightDrive.setPower(-power);
    }

    //turns robot left given a positive parameter in degrees
    public void turnLeft(double degrees) {
        right = false;
        //temporary fix for angle overshooting
        if (degrees > 60) {
            Gyrovalues.add(getRealAngle() + degrees - 30);
        } else {
            Gyrovalues.add(getRealAngle() + degrees / 2.0);
        }

        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }

    //turns robot right given a positive parameter in degrees
    public void turnRight(double degrees) {
        right = true;
        if (degrees > 60) {
            Gyrovalues.add(getRealAngle() - degrees + 30);
        } else {
            Gyrovalues.add(getRealAngle() - degrees / 2.0);
        }
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }

    //modified version of sleep function that during sleep also updates motor ending and calls endmotors
    public void startSleep(int ms) {
        int time = 0;
        while (time <= ms) {
            endMotors();
            getRealAngle();
            sleep(20);
            time += 20;
        }
    }

    //method that calculates - infinity to + infinity degree measurement of the robot
    public double getRealAngle() {
        checkJump();
        return 360 * rotcount + gyro.getAngle();
    }

    double pid() {
        if (armMotor.getCurrentPosition() > 800) return 0.15;
        else if (armMotor.getCurrentPosition() > 500) return 0.1;
        else if (armMotor.getCurrentPosition() > 200) return 0.05;
        else return 0;
    }

}