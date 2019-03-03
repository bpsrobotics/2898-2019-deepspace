package com.team2898.engine.test.test

import com.team254.lib.util.motion.MotionProfileConstraints
import com.team254.lib.util.motion.MotionProfileGenerator
import com.team254.lib.util.motion.MotionProfileGoal
import com.team254.lib.util.motion.MotionState
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.get
import com.team2898.engine.math.linear.row
import com.team2898.robot.motion.Constants
import com.team2898.robot.motion.Trajectories
import com.team2898.robot.subsystem.Arm
import io.kotlintest.specs.StringSpec
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.feetPerSecond
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import java.awt.Color
import java.io.File


class FollowerTest : StringSpec() {
    val path = Trajectories.testPath
    val tracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)
    var i = 0.0
    val dt = 0.01
    var prevPos = Pose2d(0.0.feet, 0.0.feet, 0.0.degree)

    init {
        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val refXList = arrayListOf<Double>()
        val refYList = arrayListOf<Double>()

        tracker.reset(path)

        while (!tracker.isFinished) {
            val ref = tracker.nextState(prevPos, i.second)
            prevPos = tracker.referencePoint!!.state.state.pose
            println("lin ${ref.linearVelocity.feetPerSecond}, ang. ${ref.angularVelocity.value}")
            i += dt
        }
    }

}
