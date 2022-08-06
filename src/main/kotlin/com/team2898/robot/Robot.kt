package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.commands.Safety
import com.team2898.robot.commands.commandGroups.TeleopWSafety
import com.team2898.robot.subsystem.*
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.command.Scheduler

object Robot : TimedRobot() {
    val data = byteArrayOf()
    override fun robotInit() {
        val led = DigitalOutput(2)
        AsyncLooper(50.0) {
            if (OI.driverController.getRawButton(5)) led.set(true)
            else led.set(false)
        }.start()
        CameraServer.startAutomaticCapture()
        Navx.reset()
        Arm
        Drivetrain
        DiscBrake
        Intake
        Safety
    }

    override fun autonomousInit() {
        TeleopWSafety.start()
    }

    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        TeleopWSafety.start()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }
}

fun main() {
    RobotBase.startRobot { Robot }
}