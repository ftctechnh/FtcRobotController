package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Drivetrain {

    public DcMotor left_front = null;
    public DcMotor left_back = null;
    public DcMotor right_front = null;
    public DcMotor right_back = null;

    public DcMotor intake_motor = null;
    public DcMotor intake2_motor = null;
    public DcMotor shooting_motor = null;
    public DcMotor wobble_dropper = null;
    //public Servo arm_servo = null;

    public void mechanum_hardware (HardwareMap m_hardwaremap) { //init

        left_front = m_hardwaremap.get(DcMotor.class,"left_front");
        left_back = m_hardwaremap.get(DcMotor.class,"left_back");
        right_front = m_hardwaremap.get(DcMotor.class,"right_front");
        right_back = m_hardwaremap.get(DcMotor.class,"right_back");

        intake_motor = m_hardwaremap.get(DcMotor.class, "intake_motor");
        intake2_motor = m_hardwaremap.get(DcMotor.class, "intake2_motor");
        shooting_motor = m_hardwaremap.get(DcMotor.class, "shooting_motor");
        wobble_dropper = m_hardwaremap.get(DcMotor.class, "wobble_dropper");
        //arm_servo = m_hardwaremap.get(Servo.class, "arm_servo");



        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooting_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        wobble_dropper.setDirection(DcMotorSimple.Direction.REVERSE);
        //ServoController sc = arm_servo.getController();
        //sc.pwmEnable();
        //arm_servo.setPosition(0.7);
        //arm_servo.setDirection(Servo.Direction.REVERSE);

    }


}
