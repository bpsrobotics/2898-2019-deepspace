package com.team2898.robot

import com.team2898.engine.subsystems.Navx
import com.team2898.robot.commands.testbot.TestAuto
import com.team2898.robot.commands.testbot.TestTeleop
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Scheduler

object TestBot: TimedRobot() {
    val auto = TestAuto
    override fun robotInit() {
        Drivetrain
        Navx
    }

    override fun teleopInit() {
        TestTeleop().start()
    }

    override fun autonomousInit() {
        auto.start()
    }

    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }
}

fun main(args: Array<String>) {
    RobotBase.startRobot { TestBot }
}