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


@TeleOp(name = "TeleOp Basic")

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;
    private DcMotor liftDrive = null;

        private DcMotor grabNabberLeft = null;
        private DcMotor grabNabberRight = null;

    //private Servo leftServo = null;
        //private Servo leftServo = null;
    //private Servo rightServo = null;
    private Servo armLeftServo = null;
    private Servo armRightServo = null;
//    private DigitalChannel touchSensor = null;

            /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //Hardwaremaps for all motors and servos (look on driverstation to see what to call them
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");
        grabNabberLeft = hardwareMap.get(DcMotor.class, "grab_nabber_left");
        grabNabberRight = hardwareMap.get(DcMotor.class, "grab_nabber_right");

        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
        //leftServo = hardwareMap.get(Servo.class, "left_servo");
        //rightServo = hardwareMap.get(Servo.class, "right_servo");
        armLeftServo = hardwareMap.get(Servo.class, "arm_servo_blue");
        armRightServo = hardwareMap.get(Servo.class, "arm_servo_red");
//        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_Sensor");

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

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        armLeftServo.setPosition(0.7);
        armRightServo.setPosition(0.3);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

/*
        if(gamepad1.x) {
            if(touchSensor.getState()){
                grabNabberLeft.setPower(100);

            grabNabberRight.setPower(-100);
        }}*/
        if(gamepad1.b) {
            grabNabberLeft.setPower(-1);
            grabNabberRight.setPower(1);
        }
        else if(gamepad1.x){
            grabNabberLeft.setPower(1);
            grabNabberRight.setPower(-1);
        }
        else{
            grabNabberLeft.setPower(0);
            grabNabberRight.setPower(0);
        }

        if (gamepad2.dpad_up) {
            armLeftServo.setPosition(0.7);
        }
        if (gamepad2.dpad_left) {
            armLeftServo.setPosition(0.1);

        }
        if (gamepad2.dpad_down) {
            armRightServo.setPosition(0.8);
        }
        if (gamepad2.dpad_right) {
            armRightServo.setPosition(0.3);

        }

        //forward/backward
        //Bumpers for speedboost/slow mode
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

        //slide drive
        centerDrive.setPower(gamepad1.left_stick_x * 0.8);

        //elevator code
        liftDrive.setPower(gamepad2.left_stick_y);
        telemetry.addData("encoder", liftDrive.getCurrentPosition());

        //if nothing is happening stop the drives
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //brake the elevator
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //code to grab a block
        /*if (gamepad1.a) {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }
        else if (gamepad1.x){
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }*/
        // Show the elapsed game time and wheel power.
    }
    @Override
    public void stop() {

    }


}
