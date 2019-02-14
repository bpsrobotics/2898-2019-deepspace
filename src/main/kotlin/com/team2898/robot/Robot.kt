package com.team2898.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.robot.subsystem.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import kotlinx.serialization.ImplicitReflectionSerializer

@ImplicitReflectionSerializer
object Robot : TimedRobot() {


    override fun robotInit() {
        DiscBrake
        Arm
//        ArmBackup
    }

    override fun teleopInit() {
//        Arm.updateTarget(0.5)
    }

    override fun teleopPeriodic() {
//        Intake.hatch = OI.opCtl.getRawButton(3)
//        Intake.cargoIntake(OI.opCtl.getRawAxis(5))
//        Intake.hatchIntake()
        Arm.talons(-3.8/12)
        DiscBrake.brakeUpdate()
//        if (OI.opCtl.getRawButton(0)) Arm.updateTarget(1.0)
//        if (OI.opCtl.getRawButton(2)) Arm.updateTarget(-0.5)
//        if (OI.opCtl.getRawButton(0)) Arm.updateTarget(-1.0)
//        if (OI.opCtl.getRawButton(0)) Arm.updateTarget(1.5)
    }
}

@ImplicitReflectionSerializer
fun main() {
    RobotBase.startRobot { Robot }
}