package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

public class AutomaticPickup extends OpenCvPipeline {

    Telemetry tel;
    Mat mat = new Mat();

    String dir = "null";
    String leftamt;
    String rightamt;

    int width = 320;
    int height = 240;
    int thickness = 20;

    final double THRESHOLD = 0.01;

    public AutomaticPickup(Telemetry t) {
        tel = t;
    }

    @Override
    public Mat processFrame(Mat m){
        for (int i = 0; i < width/thickness/2; i++){
            Rect left = new Rect(new Point(i*thickness, 0), new Point((i+1)*thickness, height));
            Rect right = new Rect(new Point(width - i*thickness, 0), new Point(width - (i+1)*thickness, height));

            // yellow detection code
            Imgproc.cvtColor(m, mat, Imgproc.COLOR_RGB2HSV); // range: 0-360
            Scalar lowHSV = new Scalar(20, 100, 100);
            Scalar highHSV = new Scalar(30, 255, 255);
            Core.inRange(mat, lowHSV, highHSV, mat);

            Mat left_mat = mat.submat(left);
            Mat right_mat = mat.submat(right);

            double left_white_percent = Core.sumElems(left_mat).val[0] / left.area() / 255;
            double right_white_percent = Core.sumElems(right_mat).val[0] / right.area() / 255;

            left_mat.release();
            right_mat.release();

            leftamt = left_white_percent + "";
            rightamt = right_white_percent + "";

            if (left_white_percent >= right_white_percent && left_white_percent > THRESHOLD){
                dir = ("left and i = " + i);
                break;
            } else if (left_white_percent < right_white_percent && right_white_percent > THRESHOLD){
                dir = ("right and i = " + i);
                break;
            }

        }

        // move to the item's location based on calculations

        return m;
    }

    public String getDir(){
        return dir;
    }

    public String getLeftAmount(){
        return leftamt;
    }

    public String getRightAmount(){
        return rightamt;
    }

}
