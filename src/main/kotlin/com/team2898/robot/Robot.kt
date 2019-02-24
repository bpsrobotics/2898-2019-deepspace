package com.team2898.robot

import com.team2898.robot.commands.Teleop
import com.team2898.robot.subsystem.*
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Scheduler
import kotlinx.serialization.ImplicitReflectionSerializer

@ImplicitReflectionSerializer
object Robot : TimedRobot() {
    override fun robotInit() {
        CameraServer.getInstance().startAutomaticCapture()
        Arm
        Drivetrain
        DiscBrake
        Intake
    }

//    val auto = PathFollower(profile)

    override fun autonomousInit() {
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        Teleop().start()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
        DiscBrake.brakeUpdate(false)
    }

    override fun disabledPeriodic() {
        DiscBrake.brakeUpdate(false)
    }
}

@ImplicitReflectionSerializer
fun main() {
    RobotBase.startRobot { Robot }
}