/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Linear TeleOp - 1.0", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;

    private Servo clawServo = null;
    // Carousel servo
    private CRServo cServo = null;

    // Position/encoder variables
    int MAX_ARM_POSITION = -1350;
    int MIN_ARM_POSITION = 0;
    double PID = -0.04;
    double POWER = 0.068;
    boolean armButtonPressed;

    ArmClawStateMachine sm = new ArmClawStateMachine();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // remember to check device names *****

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        cServo = hardwareMap.get(CRServo.class, "c_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        // Most robots need the motor on one` side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Initialize arm/claw to State 0 position (assuming encoder count increases as arm goes vertical)
        // Max vertical is MAX_ARM_POSITION, max horizontal is MIN_ARM_POSITION
        //armMotor.setTargetPosition(MAX_ARM_POSITION);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
/*
        while (armMotor.getCurrentPosition() > MAX_ARM_POSITION){
            armMotor.setPower(-POWER);
            telemetry.addData("Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        armMotor.setPower(PID);

 */

        // NEGATIVE POWER IS UP, ENCODERS DECREASE AS YOU MOVE THE ARM UP

        clawServo.setPosition(1.0);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //armMotor.setDirection(DcMotor.Direction.FORWARD);
        cServo.setDirection(CRServo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // DRIVING VIA POV MODE

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double power = Range.clip(drive, -0.6, 0.6);
            turn *= 0.5;
            leftPower    = Range.clip(power + turn, -0.7, 0.7) ;
            rightPower   = Range.clip(power - turn, -0.7, 0.7);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // right2Power = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            // ARM AND CLAW STATE MACHINE
            boolean up = gamepad2.y;
            boolean down = gamepad2.a;
            boolean triggerPressed = gamepad2.right_trigger >= 0.7;

            telemetry.addData("encoder value", armMotor.getCurrentPosition());

            telemetry.addData("up: ", up);
            telemetry.addData("down: ", down);

            if (up){
                armMotor.setPower(-0.3);
                telemetry.addData("Encoder: ", armMotor.getCurrentPosition());

            } else if (down){ // arm automatically comes down on it's own - gravity? or pid's not working
                armMotor.setPower(0.15);
                telemetry.addData("Encoder: ", armMotor.getCurrentPosition());

            } else armMotor.setPower(PID);

            if (triggerPressed){
                clawServo.setPosition(0);
            } else {
                clawServo.setPosition(1);
            }

            telemetry.update();

            if (armMotor.getCurrentPosition() <= MAX_ARM_POSITION && down == false) armMotor.setPower(PID);
            else if (armMotor.getCurrentPosition() >= MIN_ARM_POSITION) armMotor.setPower(PID);

            sleep(20);


            // CAROUSEL

            boolean cButtonPressedL = gamepad1.a;
            boolean cButtonPressedR = gamepad1.b;
            if (cButtonPressedL) {
                cServo.setPower(0.5);
            } else if (cButtonPressedR) {
                cServo.setPower(-0.5);
            } else {
                cServo.setPower(0);
            }

            sleep(20);

            telemetry.update();


        }
    }

    // Arm & Claw State Machine
    class ArmClawStateMachine {
        int state;
        //boolean armButtonPressed;
        boolean triggerPressed;

        public ArmClawStateMachine(){
            state = 0;
        }

        public void run(){
            triggerPressed = gamepad2.right_trigger >= 0.7;

            if (state == 0) {
                armMotor.setPower(PID);
                if (gamepad2.x){
                    state = 1;
                } else {
                    state = 0;
                }
                telemetry.addData("encoder value", armMotor.getCurrentPosition());
            } else if (state == 1) {
                armMotor.setTargetPosition(MIN_ARM_POSITION);
                while (armMotor.getCurrentPosition() < MIN_ARM_POSITION && gamepad2.x){
                    armMotor.setPower(0.15);
                    telemetry.addData("encoder value", armMotor.getCurrentPosition());
                    sleep(20);
                }
                state = 2;
                if (armMotor.getCurrentPosition() >= MIN_ARM_POSITION + 50){
                    state = 2;
                } else {
                    state = 3;
                }
                telemetry.addData("encoder value", armMotor.getCurrentPosition());
            } else if (state == 2) {
                if (triggerPressed) clawServo.setPosition(0);
                /*
                clawServo.setPosition(1);
                telemetry.addData("State", state);
                if (gamepad2.x){
                    state = 2;
                } else {
                    telemetry.addData("State", state);
                    int count = 0;
                    clawServo.setPosition(0);
                    sleep(180);
                    state = 3;
                }
                telemetry.addData("encoder value", armMotor.getCurrentPosition());

                 */
            } else if (state == 3) {
                armMotor.setTargetPosition(MAX_ARM_POSITION);
                while (armMotor.getCurrentPosition() > MAX_ARM_POSITION && !gamepad2.x){
                    armMotor.setPower(-POWER);
                    telemetry.addData("encoder value", armMotor.getCurrentPosition());
                    sleep(20);
                }
                if (armMotor.getCurrentPosition() <= MAX_ARM_POSITION) state = 4;
                else state = 1;
                telemetry.addData("encoder value", armMotor.getCurrentPosition());
            } else if (state == 4) {
                if (triggerPressed){
                    clawServo.setPosition(1);
                    state = 0;
                } else if (armButtonPressed){
                    clawServo.setPosition(1);
                    state = 1;
                } else {
                    state = 4;
                }
                telemetry.addData("encoder value", armMotor.getCurrentPosition());
            }

            telemetry.update();
        }
    }

    double pid(){
        if (armMotor.getCurrentPosition() > 800) return 0.15;
        else if (armMotor.getCurrentPosition() > 500) return 0.15;
        else if (armMotor.getCurrentPosition() > 200) return 0.13;
        else return 0.12;
    }


}
