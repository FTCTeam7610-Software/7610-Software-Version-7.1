package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous (name = "Open CV - Duck thingie Detector", group = "Auto")
//@Disabled
public class DuckTesterOpenCV extends LinearOpMode {
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

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

            //scanner.processFrame(new Mat(cameraMonitorViewId));

            telemetry.addData("Duck location", scanner.duckLocation());

            sleep(500);
        }

        cam.stopStreaming();
    }

    /*
    public class DuckScanner extends OpenCvPipeline {
        Telemetry tel;
        Mat mat = new Mat();
        int duckLocation;

        // Add new variables here
        int width = 320;
        int height = 240;

        // (0, 0) is top left of the entire camera view
        Rect LEFT_ROI = new Rect(new Point(0, 0), new Point(width / 3, height));
        Rect MIDDLE_ROI = new Rect(new Point(width / 3, 0), new Point(2 * width / 3, height));
        Rect RIGHT_ROI = new Rect(new Point(2 * width / 3, 0), new Point(width, height));

        // Constructor
        public DuckScanner(Telemetry t) {
            tel = t;
            duckLocation = 0;
        }

        // Process each frame
        @Override
        public Mat processFrame(Mat m) {
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

            Set up the threshold/rectangles for regions of the camera view
            The region with the most white percentage will be selected

            // make sure to add left and middle mat

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
                duckLocation = 1;
            } else if (right_white_percent > middle_white_percent && right_white_percent > left_white_percent) {
                telemetry.addData("Duck location", "right");
                duckLocation = 3;
            } else if (middle_white_percent > right_white_percent && middle_white_percent > left_white_percent) {
                telemetry.addData("Duck location", "middle");
                duckLocation = 2;
            } else {
                telemetry.addData("Duck location", "error");
            }

            sleep(500); // so that telemetry doesn't get overloaded


            return mat;
        }

        public int duckLocation() {
            return duckLocation;
        }

    }
    */


}