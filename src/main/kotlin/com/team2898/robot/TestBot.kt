package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.commands.testbot.TestTeleop
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Scheduler

object TestBot: TimedRobot() {
    override fun robotInit() {
        CameraServer.getInstance().startAutomaticCapture()
        Drivetrain
        Navx
        val LED = DigitalOutput(0)
        AsyncLooper(50.0) {
            if (OI.driverController.getRawButton(5)) LED.set(true)
            else LED.set(false)
        }.start()
    }

    override fun teleopInit() {
        TestTeleop().start()
    }

    override fun autonomousInit() {
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }
}

fun main(args: Array<String>) {
    RobotBase.startRobot { TestBot }
}