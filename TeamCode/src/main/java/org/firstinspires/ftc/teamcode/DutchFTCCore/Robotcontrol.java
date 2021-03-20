package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class Robotcontrol {

    public void mechanumdrive(Drivetrain m_drivetrain, double xDirection, double yDirection) { //Periodic

        double r = Math.hypot(xDirection,yDirection);
        double robotAngle = Math.atan2(yDirection,xDirection) - Math.PI/4;
        double rightX = xDirection;
        final double LeftFront = r * Math.cos(robotAngle)-rightX;
        final double RightFront = r * Math.sin(robotAngle)+rightX;
        final double LeftBack = r * Math.sin(robotAngle)-rightX;
        final double RightBack = r * Math.cos(robotAngle)+rightX;




//        double HeadingAngle = Math.atan2(xDirection, yDirection);
//        double speed = Math.sqrt(Math.pow(xDirection, 2) + Math.pow(yDirection, 2));
//
//        double RightFront = speed * Math.cos((90 - HeadingAngle - 45) / 180 * Math.PI);
//        double RightBack = speed * Math.cos((90 - HeadingAngle - 135) / 180 * Math.PI);
//        double LeftBack = speed * Math.cos((90 - HeadingAngle - 225) / 180 * Math.PI);
//        double LeftFront = speed * Math.cos((90 - HeadingAngle - 315) / 180 * Math.PI);
//
        m_drivetrain.right_front.setPower(RightFront);
        m_drivetrain.right_back.setPower(RightBack);
        m_drivetrain.left_back.setPower(LeftBack);
        m_drivetrain.left_front.setPower(LeftFront);


    }
    public void intake(Drivetrain m_Drivetrain, boolean intake){
        if (intake) {
            m_Drivetrain.intake_motor.setPower(1);
            m_Drivetrain.intake2_motor.setPower(0.45);
        } else{
            m_Drivetrain.intake_motor.setPower(0);
            m_Drivetrain.intake2_motor.setPower(0);
        }
    }

    public void shooting(Drivetrain m_Drivetrain, boolean shooting){
        if (shooting) {
            m_Drivetrain.shooting_motor.setPower(1);
        } else{
            m_Drivetrain.shooting_motor.setPower(0);
        }
    }
}
