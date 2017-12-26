package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by main2 on 10/20/2017.
 */

@Autonomous(name = "testingEncoders")
public class testingEncoders extends LinearOpMode{

    private Servo kicker = null;


    @Override
    public void runOpMode() throws InterruptedException {
        kicker = hardwareMap.get(Servo.class, "kicker");

        waitForStart();
        while (opModeIsActive()){
            kicker.setPosition(0.5);
            Thread.sleep(1000);
            kicker.setPosition(0);
        }
    }

}
