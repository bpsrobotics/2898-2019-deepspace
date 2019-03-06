package com.team2898.robot

import com.team2898.robot.commands.Teleop
import com.team2898.robot.commands.testbot.TestAuto
import com.team2898.robot.commands.testbot.TestTeleop
import com.team2898.robot.motion.Constants.kDriveBeta
import com.team2898.robot.motion.Constants.kDriveZeta
import com.team2898.robot.motion.Trajectories
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Scheduler
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.feetPerSecond
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second

object TestBot: TimedRobot() {

    override fun robotInit() {
        Drivetrain
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