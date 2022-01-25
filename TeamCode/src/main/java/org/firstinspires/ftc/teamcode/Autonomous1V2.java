package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import CupertinoRobotics.support.Hardware.Gyro;

@Autonomous(name="Autonomous1V2")
//@Disabled
public class Autonomous1V2 extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private CRServo carouselMotor=null;
    private Servo clawServo = null;
    private Gyro gyro=null;

    final int MAX_ARM_POSITION = -1350;
    final int MIDDLE_ARM_POSITION = -800;
    final int BOTTOM_ARM_POSITION = -300;
    final int MIN_ARM_POSITION = 0;

    final double DRIVE_POWER = 0.1;
    final double ARM_POWER = 0.15;
    final double PID = -0.04;

    OpenCvCamera cam;
    final double tpi = 46;
    int duckLocation = 3;
    double rotcount=0;
    boolean right=false;
    boolean angleneg=false;
    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        carouselMotor = hardwareMap.get(CRServo.class,"c_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        gyro=new Gyro(hardwareMap, AngleUnit.DEGREES);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // OPEN CV STUFF
        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
         //handle if cam return null?

        DuckScanner scanner = new DuckScanner(telemetry);
        cam.setPipeline(scanner);

//        cam.openCameraDeviceAsync(
//                () -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
//        );

        // setting up the camera device and streaming system
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened(){
                cam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                //please die
            }
        });

        clawServo.setPosition(1);

        waitForStart();

        while(opModeIsActive()) {
            forward(23);
            /*
            clawServo.setPosition(0); // close claw
            forward(15);
            // scan ducks
            turnRight(90);
            leftDrive.setPower(0.5);
            rightDrive.setPower(0.5);
            sleep(500);
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            forward(23);
            turnLeft(90);
            leftDrive.setPower(-0.5);
            rightDrive.setPower(-0.5);
            sleep(500);
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            forward(10);
            if (duckLocation == 1) { // left location - 1st level
                armMotor.setTargetPosition(BOTTOM_ARM_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (armMotor.getCurrentPosition() > BOTTOM_ARM_POSITION) {
                    armMotor.setPower(-ARM_POWER);
                    sleep(20);
                }
                armMotor.setPower(PID);
                // do we have to set power to pid here too?
            } else if (duckLocation == 2) { // right location - 3rd level
                armMotor.setTargetPosition(MIDDLE_ARM_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (armMotor.getCurrentPosition() > MIDDLE_ARM_POSITION) {
                    armMotor.setPower(-ARM_POWER);
                    sleep(20);
                }
                armMotor.setPower(PID);
            } else if (duckLocation == 3) { // middle location - 2nd level
                armMotor.setTargetPosition(MAX_ARM_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (armMotor.getCurrentPosition() > MAX_ARM_POSITION) {
                    armMotor.setPower(-ARM_POWER);
                    sleep(20);
                }
                armMotor.setPower(PID);
            }
            clawServo.setPosition(1);
            sleep(20);
            backward(8);
            // bring arm down & close claw
            armMotor.setTargetPosition(MIN_ARM_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (armMotor.getCurrentPosition() < MIN_ARM_POSITION) {
                armMotor.setPower(ARM_POWER);
                sleep(20);
            }
            armMotor.setPower(PID);
            clawServo.setPosition(0);
            backward(5);
            // parking in alliance storage unit
            turnLeft(90);
            leftDrive.setPower(-0.5);
            rightDrive.setPower(-0.5);
            sleep(500);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            forward(38);
            // turn right 45
            leftDrive.setPower(0.5);
            rightDrive.setPower(0.5);
            sleep(250);
            leftDrive.setPower(0);
            rightDrive.setPower(0);

             */
        }
    }

    public void backward(double inches){
        double distanceInTicks = inches*tpi;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition()-distanceInTicks));
        rightDrive.setTargetPosition((int) (rightDrive.getCurrentPosition()-distanceInTicks));

        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);
    }

    public void forward(double inches){
        double distanceInTicks = inches*tpi;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition() + distanceInTicks));
        rightDrive.setTargetPosition((int) (rightDrive.getCurrentPosition() + distanceInTicks));

        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);
    }

    public class DuckScanner extends OpenCvPipeline {
        Telemetry tel;
        Mat mat = new Mat();


        // Add new variables here
        int width = 320;
        int height = 240;

        // (0, 0) is top left of the entire camera view
        Rect LEFT_ROI = new Rect(new Point(0, 0), new Point(width/3, height));
        Rect MIDDLE_ROI = new Rect(new Point(width/3, 0), new Point(2*width/3, height));
        Rect RIGHT_ROI = new Rect(new Point(2*width/3, 0), new Point(width, height));

        // Constructor
        public DuckScanner(Telemetry t){
            tel = t;
        }
        // Process each frame
        @Override
        public Mat processFrame(Mat m){
            // m is the RGB matrix that the camera sees
            // Converting matrix from RGB --> HSV
            // HSV: Hue - color, Saturation - intensity, value - brightness
            Imgproc.cvtColor(m, mat, Imgproc.COLOR_RGB2HSV); // range: 0-360
            // Creating an HSV range to detect orange (duck color)
            // Will only be considered yellow if all three values are within this range
            // chart: http://www.workwithcolor.com/orange-brown-color-hue-range-01.htm
            // Scalar lowHSV = new Scalar(24, 100, 50);
            // Scalar highHSV = new Scalar(39, 100, 85);
            Scalar lowHSV = new Scalar(20, 100, 100);
            Scalar highHSV = new Scalar(30, 255, 255);
            // 20, 100, 100 - low
            // 30, 255, 255 - high

            // Thresholding - showing the part of the image that is yellow
            // Parameters: source matrix, lower bound, higher bound, destination matrix
            // After this, regions with HSV will be white, bg black
            Core.inRange(mat, lowHSV, highHSV, mat);
            /*
            Set up the threshold/rectangles for regions of the camera view
            The region with the most white percentage will be selected
             */
            Mat left_mat = mat.submat(LEFT_ROI);
            Mat middle_mat = mat.submat(MIDDLE_ROI);
            Mat right_mat = mat.submat(RIGHT_ROI);

            double left_white_percent = Core.sumElems(left_mat).val[0] / LEFT_ROI.area() / 255;
            double middle_white_percent = Core.sumElems(middle_mat).val[0] / LEFT_ROI.area() / 255;
            double right_white_percent = Core.sumElems(right_mat).val[0] / RIGHT_ROI.area() / 255;


            //double left_white_percent = Core.mean(left_mat).val[0] / 255;
            //double middle_white_percent = Core.mean(middle_mat).val[0] / 255;
            //double right_white_percent = Core.mean(right_mat).val[0] / 255;

            // to prevent memory leaks
            left_mat.release();
            right_mat.release();
            middle_mat.release();

            telemetry.addData("left white percent", left_white_percent);
            telemetry.addData("middle white percent", middle_white_percent);
            telemetry.addData("right white percent", right_white_percent);

            telemetry.addData("stuff", LEFT_ROI.area());
            telemetry.addData("stuff 2", Core.sumElems(left_mat).val[0]);
            //telemetry.addData("middle white", middle_white_percent);
            //telemetry.addData("right white", right_white_percent);

            if (left_white_percent > middle_white_percent && left_white_percent > right_white_percent) {
                telemetry.addData("Duck location", "left");
                duckLocation = 1;
            } else if (right_white_percent > middle_white_percent && right_white_percent > left_white_percent){
                telemetry.addData("Duck location", "right");
                duckLocation = 2;
            } else if (middle_white_percent > right_white_percent && middle_white_percent > left_white_percent){
                telemetry.addData("Duck location", "middle");
                duckLocation = 3;
            } else {
                telemetry.addData("Duck location", "error");
            }

            sleep(500); // so that telemetry doesn't get overloaded


            return mat;
        }

    }
    public void turnLeft(double degrees){
        right=false;
        leftDrive.setPower(-DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);
        while(getRealAngle()<=degrees){
            checkJump();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void turnRight(double degrees){
        right=true;
        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(-DRIVE_POWER);
        while(getRealAngle()>=degrees){
            checkJump();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public double getRealAngle(){
        checkJump();
        return 360*rotcount+gyro.getAngle();
    }
    public void checkJump(){
        if(right){
            if(angleneg){
                if(gyro.getAngle()>=0){
                    rotcount-=1;
                    angleneg=false;
                }
            }
            if(gyro.getAngle()<0){
                angleneg=true;
            }
        }else{
            if(!angleneg){
                if(gyro.getAngle()<0){
                    rotcount+=1;
                    angleneg=true;
                }
            }
            if(gyro.getAngle()>=0){
                angleneg=false;
            }
        }
    }
}
