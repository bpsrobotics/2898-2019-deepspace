package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.commands.Safety
import com.team2898.robot.commands.Teleop
import com.team2898.robot.commands.commandGroups.TeleopWSafety
import com.team2898.robot.subsystem.*
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.serialization.ImplicitReflectionSerializer

@ImplicitReflectionSerializer
object Robot : TimedRobot() {
    override fun robotInit() {
        CameraServer.getInstance().startAutomaticCapture()
        Navx.reset()
        Arm
        Drivetrain
        DiscBrake
        Intake
    }
    override fun autonomousInit() {
        TeleopWSafety.start()
    }

    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        Teleop.start()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
//        DiscBrake.brakeUpdate(false)
    }

    override fun disabledPeriodic() {
//        DiscBrake.brakeUpdate(false)
    }
}

@ImplicitReflectionSerializer
fun main() {
    RobotBase.startRobot { Robot }
}