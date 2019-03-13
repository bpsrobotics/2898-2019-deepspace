package com.team2898.engine.test.test

import arrow.core.left
import com.team254.lib.util.motion.MotionProfileConstraints
import com.team254.lib.util.motion.MotionProfileGenerator
import com.team254.lib.util.motion.MotionProfileGoal
import com.team254.lib.util.motion.MotionState
import com.team2898.engine.math.linear.*
import com.team2898.robot.Localizer
import com.team2898.robot.config.DrivetrainConf.Dt_A
import com.team2898.robot.config.DrivetrainConf.Dt_Kc
import com.team2898.robot.config.DrivetrainConf.Dt_Kff
import com.team2898.robot.motion.Trajectories
import edu.wpi.first.wpilibj.command.WaitCommand
import io.kotlintest.specs.StringSpec
import org.apache.commons.math3.linear.RealMatrix
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.feetPerSecond
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput
import java.io.File

class FollowerTest : StringSpec() {
    private val kBeta = 2.0
    private val kZeta = 0.85
    init {
        var t = 0.0
        val dt = 0.01
        val path = Trajectories.testPath
        val itr = path.iterator()
        val test = MotionProfileGenerator.generateProfile(MotionProfileConstraints(3.0, 2.0), MotionProfileGoal(6.0), MotionState(0.0, 0.0, 0.0, 0.0))
        val sb = StringBuilder()
        var prev = 0.0
        sb.append("t, u, v\n")
        while (t < test.endTime()) {
            t += dt
            val state = test.stateByTimeClamped(t).vel()
            val nextState = test.stateByTimeClamped(t + dt).vel()
            val r = Matrix(arrayOf(row(state), row(state)))
            val nextR = Matrix(arrayOf(row(nextState), row(nextState)))
            val x = Matrix(arrayOf(row(prev), row(prev)))
            val u = genU(r, nextR, x = x)
            prev = state
            sb.append("$t, ${u[0, 0]}, $state\n")
        }
        File("test.csv").writeText(sb.toString())
    }

    fun genU(r: Matrix, r_next: Matrix, ff: Boolean = true, x: RealMatrix): Matrix {
        val u_feedback = Matrix((Dt_Kc * (r - x)).data)
        if (!ff) return u_feedback
        val u_feedforward = Dt_Kff * (r_next - Dt_A * r)
        val uReturn = Matrix((u_feedback + u_feedforward).data)
        return uReturn
    }
}
