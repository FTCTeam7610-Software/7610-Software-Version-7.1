package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import java.util.ArrayList;
import java.util.List;

//import CupertinoRobotics.support.Hardware.Gyro;

@Autonomous(name="Autonomous")
public class Autonomous1 extends LinearOpMode{

    //hardware components
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private CRServo carouselMotor=null;
    //private Gyro gyro=null;
    private BNO055IMU imu;

    //subject to change after rigorous testing
    final double tpi = 46;
    final int MAX_ARM_POSITION = -1350;
    final int MIDDLE_ARM_POSITION = -800;
    final int BOTTOM_ARM_POSITION = -300;
    final int MIN_ARM_POSITION = 0;
    final double tpd = (MIN_ARM_POSITION-MAX_ARM_POSITION)/120.0;
    final double power = 0.25;
    final double armPower=0.15;

    private Servo clawServo = null;
    // Carousel servo
    private CRServo cServo = null;

    // Position/encoder variables
    double PID = -0.04;
    boolean armButtonPressed;



    //states
    boolean forward=false;
    boolean right=false;
    boolean ArmUp=false;

    //helper variables
    List<Double> leftencodervalues=new ArrayList<Double>();
    List<Double> rightencodervalues=new ArrayList<Double>();
    List<Double> armencodervalues=new ArrayList<Double>();
    List<Double> Gyrovalues=new ArrayList<Double>();
    List<Double> servotimevalues=new ArrayList<Double>();
    double rotcount = 0;
    boolean angleneg=false;

    OpenCvCamera cam;
    int locationDuck = 2;

    @Override
    public void runOpMode() {
        //hardware initialization
        imu=hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params= new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        carouselMotor=hardwareMap.get(CRServo.class,"c_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        //gyro=new Gyro(hardwareMap, AngleUnit.DEGREES);

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
            public void onOpened(){
                cam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
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
            // duck location is the variable that says where the duck was
            // use that to determine where to place our shipping element
            //scanner.processFrame(new Mat(cameraMonitorViewId));

            //this code moves robot from starting to in front of the shipping hub and drops freight on the middle level
            clawServo.setPosition(0);
            sleep(1500);

            if (locationDuck == 1){ // left location - bottom level
                if (true){
                    armMotor.setTargetPosition(BOTTOM_ARM_POSITION);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (armMotor.getCurrentPosition() > BOTTOM_ARM_POSITION){
                        armMotor.setPower(-armPower);
                        telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                        telemetry.update();
                        sleep(20);
                    }
                    armMotor.setPower(PID);
                }
            } else if (locationDuck == 2){ // right spot - top level
                if (true){
                    armMotor.setTargetPosition(MAX_ARM_POSITION);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (armMotor.getCurrentPosition() > MAX_ARM_POSITION){
                        armMotor.setPower(-armPower);
                        telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                        telemetry.update();
                        sleep(20);
                    }
                    armMotor.setPower(PID);
                }
            } else if (locationDuck == 3){ // middle spot - middle level
                if (true){
                    armMotor.setTargetPosition(MIDDLE_ARM_POSITION);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (armMotor.getCurrentPosition() > MIDDLE_ARM_POSITION){
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

            if (locationDuck == 1){ //bottom
                turnRight(48);
                startSleep(1000);
                forward(19);
                startSleep(1500);
            } else if (locationDuck == 2){ //top
                turnRight(36);
                startSleep(1000);
                forward(28);
                startSleep(1500);
            } else if (locationDuck == 3){ //middle
                turnRight(48);
                startSleep(1000);
                forward(22);
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

            if (locationDuck == 1){ //bottom
                turnRight(30);
                startSleep(800);
                backward(45);
                startSleep(2000);
                carouselMotor.setPower(0.25);
                sleep(1000);
                carouselMotor.setPower(0);
            } else if (locationDuck == 2){ //top
                turnRight(23);
                startSleep(800);
                backward(43);
                startSleep(2400);
                carouselMotor.setPower(0.25);
                leftDrive.setPower(-0.01);
                rightDrive.setPower(-0.01);
                sleep(8000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                carouselMotor.setPower(0);
            } else if (locationDuck == 3){ //middle
                turnRight(30);
                startSleep(800);
                backward(45);
                startSleep(2000);
                carouselMotor.setPower(0.25);
                sleep(1000);
                carouselMotor.setPower(0);
            }

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDrive.setPower(power);
            rightDrive.setPower(power);
            turnLeft(60);
            startSleep(1000);
            forward(20);
            startSleep(1000);
            // parking in alliance shipping hub
            //turnLeft(45);
            //forward(13);
            // parking in warehouse
            /* turnRight(50);
            forward(20);
            turnLeft(5);
            forward(65);
            */

            /*
            //goes to warehouse (or should, havent tested out the final couple lines)
            //turnLeft(110);
            //startSleep(2000);
            //forward(20);
            //startSleep(2000);
            //turnRight(90);
            //startSleep(3000);
            //forward(25);
            //startSleep(2000);
            //turnRight(90);
            //startSleep(1000);
            //forward(70);
            //startSleep(2500);

             */


            telemetry.addLine("Scanning");
            telemetry.update();


            sleep(500);

            break;
        }
    }
    //spins servo 7 seconds per duck (hardcoded in)
    public void spinCarousel(int numducks){
        carouselMotor.setPower(0.5);
        double time=getRuntime();
        servotimevalues.add(time+7000*numducks);
    }
    /*
    //Helps calculate continous angle value (- infinity to + infinity by checking when the angle switches signs
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
            if(gyro.getAngle()>0){


                angleneg=false;
            }
        }
    }

     */
    //Method that helps asynchronous movement and motor access by checking all the motors and seeing wether they should be ended or not
    public void endMotors(){
        if(servotimevalues.size()>0){
            if(servotimevalues.get(0)>=getRuntime()){
                carouselMotor.setPower(0);
                servotimevalues.remove(0);
            }
        }
        if(forward){
            if(leftencodervalues.size()>0){
                if(leftDrive.getCurrentPosition()>=leftencodervalues.get(0)){
                    leftencodervalues.remove(0);
                    leftDrive.setPower(0);

                }
            }
            if(rightencodervalues.size()>0){
                if(rightDrive.getCurrentPosition()>=rightencodervalues.get(0)){
                    rightencodervalues.remove(0);
                    rightDrive.setPower(0);
                }
            }


        }else{
            if(leftencodervalues.size()>0){
                if(leftDrive.getCurrentPosition()<=leftencodervalues.get(0)){
                    leftencodervalues.remove(0);
                    leftDrive.setPower(0);

                }
            }
            if(rightencodervalues.size()>0){
                if(rightDrive.getCurrentPosition()<=rightencodervalues.get(0)){
                    rightencodervalues.remove(0);
                    rightDrive.setPower(0);
                }
            }


        }
        if(right){
            if(Gyrovalues.size()>0){
                if(getRealAngle()<=Gyrovalues.get(0)){
                    Gyrovalues.remove(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(0);
                }else{
                    leftDrive.setPower(power);
                    rightDrive.setPower(-power);
                }
            }
        }else{
            if(Gyrovalues.size()>0){
                if(getRealAngle()>=Gyrovalues.get(0)){
                    Gyrovalues.remove(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(0);
                }else{
                    leftDrive.setPower(-power);
                    rightDrive.setPower(power);
                }
            }
        }
        if(ArmUp){
            if(armencodervalues.size()>0){
                if(armMotor.getCurrentPosition()<=armencodervalues.get(0)){
                    armMotor.setPower(0);
                    armencodervalues.remove(0);
                }
            }
        }else {
            if (armencodervalues.size() > 0) {
                if (armMotor.getCurrentPosition() >= armencodervalues.get(0)) {
                    armMotor.setPower(0);
                    armencodervalues.remove(0);
                }
            }
        }
    }
    //moves arm up given a positive parameter in degrees
    public void movingArmUp(int degrees){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmUp=true;
        if(armMotor.getCurrentPosition()-degrees*tpd<MAX_ARM_POSITION){
            armencodervalues.add((double)MAX_ARM_POSITION);
        }
        else{
            armencodervalues.add(armMotor.getCurrentPosition()-degrees*tpd);
        }
        armMotor.setPower(-armPower);
    }
    //moves arm down given a positive parameter in degrees
    public void movingArmDown(int degrees){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmUp=false;
        if(armMotor.getCurrentPosition()+degrees*tpd>MIN_ARM_POSITION){
            armencodervalues.add((double)MIN_ARM_POSITION);
        }
        else{
            armencodervalues.add(armMotor.getCurrentPosition()+degrees*tpd);
        }
        armMotor.setPower(armPower);
    }
    //moves robot forward given a positive parameter in inches
    //temporarily ticks
    public void forward(double inches){
        forward=true;
        double distanceInTicks = inches*tpi;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftencodervalues.add(leftDrive.getCurrentPosition()+distanceInTicks);
        rightencodervalues.add(rightDrive.getCurrentPosition()+distanceInTicks);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }
    //moves robot backward given a positive parameter in inches
    public void backward(double inches){
        forward=false;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double distanceInTicks = inches*tpi;
        leftencodervalues.add(leftDrive.getCurrentPosition()-distanceInTicks);
        rightencodervalues.add(rightDrive.getCurrentPosition()-distanceInTicks);
        leftDrive.setPower(-power);
        rightDrive.setPower(-power);
    }
    //turns robot left given a positive parameter in degrees
    public void turnLeft(double degrees){
        right=false;
        //temporary fix for angle overshooting
        if(degrees>60){
            Gyrovalues.add(getRealAngle()+degrees-30);
        }else{
            Gyrovalues.add(getRealAngle()+degrees/2.0);
        }

        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }
    //turns robot right given a positive parameter in degrees
    public void turnRight(double degrees){
        right=true;
        if(degrees>60){
            Gyrovalues.add(getRealAngle()-degrees+30);
        }else{
            Gyrovalues.add(getRealAngle()-degrees/2.0);
        }
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }
    //modified version of sleep function that during sleep also updates motor ending and calls endmotors
    public void startSleep(int ms){
        int time=0;
        while(time<=ms){
            endMotors();
            getRealAngle();
            sleep(20);
            time+=20;
        }
    }


    //method that calculates - infinity to + infinity degree measurement of the robot
    public double getRealAngle(){
        //checkJump();
        return 360*rotcount+0; //gyro.getAngle();
    }

    // OPEN CV CLASS

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


//            double left_white_percent = Core.mean(left_mat).val[0] / 255;
//            double middle_white_percent = Core.mean(middle_mat).val[0] / 255;
//            double right_white_percent = Core.mean(right_mat).val[0] / 255;

            // to prevent memory leaks
            left_mat.release();
            right_mat.release();
            middle_mat.release();

            telemetry.addData("left white percent", left_white_percent);
            telemetry.addData("middle white percent", middle_white_percent);
            telemetry.addData("right white percent", right_white_percent);

//            telemetry.addData("stuff", LEFT_ROI.area());
//            telemetry.addData("stuff 2", Core.sumElems(left_mat).val[0]);
            //telemetry.addData("middle white", middle_white_percent);
            //telemetry.addData("right white", right_white_percent);

            if (left_white_percent > middle_white_percent && left_white_percent > right_white_percent) {
                telemetry.addData("Duck location", "left");
                locationDuck = 1;
            } else if (right_white_percent > middle_white_percent && right_white_percent > left_white_percent){
                telemetry.addData("Duck location", "right");
                locationDuck = 2;
            } else if (middle_white_percent > right_white_percent && middle_white_percent > left_white_percent){
                telemetry.addData("Duck location", "middle");
                locationDuck = 3;
            } else {
                telemetry.addData("Duck location", "error");
            }

            sleep(500); // so that telemetry doesn't get overloaded


            return mat;
        }

    }

    // ARM CODE

//    class ArmClawStateMachine {
//        int state;
//        //boolean armButtonPressed;
//
//        public ArmClawStateMachine(){
//            state = 0;
//        }
//
//        public void run(){
//
//            if (state == 0) {
//                armMotor.setPower(0);
//                if (true){
//                    state = 1;
//                } else {
//                    state = 0;
//                }
//                telemetry.addData("encoder value", armMotor.getCurrentPosition());
//            } else if (state == 1) {
//                armMotor.setTargetPosition(MAX_ARM_POSITION);
//                while (armMotor.getCurrentPosition() < MIN_ARM_POSITION && gamepad2.x){
//                    armMotor.setPower(pid());
//                    clawServo.setPosition(1);
//                    telemetry.addData("encoder value", armMotor.getCurrentPosition());
//                }
//                if (armMotor.getCurrentPosition() >= MIN_ARM_POSITION){
//                    state = 2;
//                } else {
//                    state = 3;
//                }
//                telemetry.addData("encoder value", armMotor.getCurrentPosition());
//            } else if (state == 2) {
//                clawServo.setPosition(1);
//                telemetry.addData("State", state);
//                if (gamepad2.x){
//                    state = 2;
//                } else {
//                    telemetry.addData("State", state);
//
//
//                    int count = 0;
//                    clawServo.setPosition(0);
//                    while (count < 30){
//                        sleep(20);
//                        telemetry.addData("count", count);
//                        count++;
//                    }
//                    state = 3;
//                }
//                telemetry.addData("encoder value", armMotor.getCurrentPosition());
//            } else if (state == 3) {
//                armMotor.setTargetPosition(MAX_ARM_POSITION);
//                while (armMotor.getCurrentPosition() > MAX_ARM_POSITION && !gamepad2.x){
//                    //armMotor.setPower(-POWER);
//                    telemetry.addData("encoder value", armMotor.getCurrentPosition());
//                }
//                if (armMotor.getCurrentPosition() <= MAX_ARM_POSITION) state = 4;
//                else state = 1;
//                telemetry.addData("encoder value", armMotor.getCurrentPosition());
//            } else if (state == 4) {
//                if (triggerPressed){
//                    clawServo.setPosition(1);
//                    state = 0;
//                } else if (armButtonPressed){
//                    clawServo.setPosition(1);
//                    state = 1;
//                } else {
//                    state = 4;
//                }
//                telemetry.addData("encoder value", armMotor.getCurrentPosition());
//            }
//
//            telemetry.update();

//        }
//    }

    double pid(){
        if (armMotor.getCurrentPosition() > 800) return 0.15;
        else if (armMotor.getCurrentPosition() > 500) return 0.1;
        else if (armMotor.getCurrentPosition() > 200) return 0.05;
        else return 0;
    }



}