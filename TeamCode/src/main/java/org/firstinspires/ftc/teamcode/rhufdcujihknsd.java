package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class Robotcontrol {

    //Periodic
    public void autonome(Drivetrain m_drivetrain) {

//

        m_drivetrain.right_front.setPower(1);
        m_drivetrain.right_back.setPower(1);
        m_drivetrain.left_back.setPower(1);
        m_drivetrain.left_front.setPower(1);


    }
