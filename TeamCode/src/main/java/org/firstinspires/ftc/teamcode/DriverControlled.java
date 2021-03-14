package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.MovementSubSystem;

@TeleOp( name = "DriverControlled")
public class DriverControlled extends OpMode {

    Robot bot;

    @Override
    public void init() {
        bot = new Robot(this);
        bot.StartGuidanceSubSystem();
        bot.StartIMUSubSystem();
        bot.StartMovementSubSystem();
        bot.MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bot.MotorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bot.Calibrate();
    }

    @Override
    public void loop() {
        MovementSubSystem.yMov = gamepad1.left_stick_y;
        MovementSubSystem.xMov = gamepad1.left_stick_x;
        MovementSubSystem.rotation = gamepad1.right_trigger -gamepad1.left_trigger;
        bot.Update();

    }
}
