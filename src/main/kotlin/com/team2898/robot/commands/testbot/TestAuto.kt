package com.team2898.robot.commands.testbot

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.motion.Trajectories
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker

object TestAuto: Command() {
    private val kBeta = 2.0
    private val kZeta = 0.85
    val tracker = RamseteTracker(kBeta, kZeta)
    val path = Trajectories.testPath

    override fun initialize() {
        tracker.reset(path)

    }

    override fun isFinished(): Boolean {
        return tracker.isFinished
    }

    override fun execute() {
    }

}