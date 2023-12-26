package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JustinTeleOp")
public class JustinTeleOp extends LinearOpMode {

  private DcMotor Motor1;
  private DcMotor Motor2;
  private DcMotor Motor3;
  private Servo Servo1;

  /**
   
This function is executed when this Op Mode is selected from the Driver Station.*/@Override
public void runOpMode() {
  Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
  Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
  Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
  Servo1 = hardwareMap.get(Servo.class, "Servo1");

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // M1: Left M2: Right M3: Crane S1: 
        Motor1.setPower(gamepad1.left_stick_y / 2);
        Motor2.setPower(-gamepad1.right_stick_y / 2);

        //if d-pad up is clicked
        if (gamepad2.dpad_up)                       //
          Motor3.setPower(-0.8); //Move motor3 up
        else Motor3.setPower(0);

        //if d-pad down is clicked
        if (gamepad2.dpad_down) //Move motor3 down             //
          Motor3.setPower(0.8);
        else Motor3.setPower(0);

        if (gamepad2.a)                                                 //
          Servo1.setPosition(Servo1.getPosition() + 0.1);
        else Servo1.setPosition(Servo1.getPosition());

        if (gamepad2.b)                                           //
          Servo1.setPosition(Servo1.getPosition() - 0.1);
        else Servo1.setPosition(Servo1.getPosition());

        telemetry.addData("Servo Position", Servo1.getPosition());
        telemetry.update();
      }
    }
  }
}