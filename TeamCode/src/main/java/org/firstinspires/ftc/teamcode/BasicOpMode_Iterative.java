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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp Basic")

public class BasicOpMode_Iterative extends OpMode
    {
        // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;
    private DcMotor liftDrive = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo armLeftServo = null;
    private Servo armRightServo = null;

        /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");

        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        armLeftServo = hardwareMap.get(Servo.class, "arm_servo_blue");
        armRightServo = hardwareMap.get(Servo.class, "arm_servo_red");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //forward/backward

        if (gamepad1.left_bumper){
        leftDrive.setPower(-gamepad1.left_stick_y * 1);
        rightDrive.setPower(gamepad1.left_stick_y * 1);
        }
        else if (gamepad1.right_bumper){
            leftDrive.setPower(-gamepad1.left_stick_y * 0.4);
            rightDrive.setPower(gamepad1.left_stick_y * 0.4);
        }
        else{
            leftDrive.setPower(-gamepad1.left_stick_y * 0.8);
            rightDrive.setPower(gamepad1.left_stick_y * 0.8);
        }
        //turn
        leftDrive.setPower(gamepad1.right_stick_x * 0.8);
        rightDrive.setPower(gamepad1.right_stick_x * 0.8);

        centerDrive.setPower(gamepad1.left_stick_x * 0.8);

        liftDrive.setPower(gamepad2.left_stick_y);
        telemetry.addData("encoder", liftDrive.getCurrentPosition());

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*        if (gamepad2.a){
            liftDrive.setTargetPosition(0);
            liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftDrive.setPower(0.7);
            while (liftDrive.isBusy()){

            }
            liftDrive.setPower(0);
            liftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (gamepad2.x){
            liftDrive.setTargetPosition(-3300);
            liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftDrive.setPower(0.7);
            while (liftDrive.isBusy()){

            }
            liftDrive.setPower(0);
            liftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (gamepad2.b){
            liftDrive.setTargetPosition(-7000);
            liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftDrive.setPower(0.7);
            while (liftDrive.isBusy()){

            }
            liftDrive.setPower(0);
            liftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/
/*        if (gamepad2.a){
            armLeftServo.setPosition(0.95);
        }
        else
            armLeftServo.setPosition(0.05);

        if (gamepad2.x){
            armRightServo.setPosition(0.05);
        }
        else{
            armLeftServo.setPosition(0.95);
        }*/


        if (gamepad1.a) {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }
        else if (gamepad1.x){
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }


        // Show the elapsed game time and wheel power.
    }
    @Override
    public void stop() {
    }


}
