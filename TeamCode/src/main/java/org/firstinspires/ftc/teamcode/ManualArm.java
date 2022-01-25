package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ManualArm", group="Linear Opmode")
//@Disabled
public class ManualArm extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor = null;
    private Servo clawServo = null;

    // Position/encoder variables
    int MAX_ARM_POSITION = -1350;
    int MIN_ARM_POSITION = 0;
    double POWER = 0.15;
    double PID = -0.04;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        //armMotor.setTargetPosition(MAX_ARM_POSITION);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean up = gamepad1.y;
            boolean down = gamepad1.a;
            boolean triggerPressed = gamepad1.right_trigger >= 0.7;

            telemetry.addData("encoder value", armMotor.getCurrentPosition());

            telemetry.addData("up: ", up);
            telemetry.addData("down: ", down);
            telemetry.update();

            if (up){
                armMotor.setPower(-0.3);
                telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                telemetry.update();
            } else if (down){
                armMotor.setPower(POWER);
                telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
                telemetry.update();
            } else armMotor.setPower(0);

            if (triggerPressed){
                clawServo.setPosition(0);
            } else {
                clawServo.setPosition(1);
            }

            if (armMotor.getCurrentPosition() <= MAX_ARM_POSITION) armMotor.setPower(PID);
            else if (armMotor.getCurrentPosition() >= MIN_ARM_POSITION) armMotor.setPower(PID);

            sleep(20);
        }

    }
}
