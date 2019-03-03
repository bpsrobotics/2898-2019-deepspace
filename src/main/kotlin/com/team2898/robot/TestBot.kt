package com.team2898.robot

import com.team2898.robot.motion.Constants.kDriveBeta
import com.team2898.robot.motion.Constants.kDriveZeta
import com.team2898.robot.motion.Trajectories
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
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

object TestBot {
    val path = Trajectories.testPath
    val tracker = RamseteTracker(kDriveBeta, kDriveZeta)
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
            println("lin ${ref.linearVelocity}, ang. ${ref.angularVelocity}")
        }

    }

}

fun main(args: Array<String>) {
//    RobotBase.startRobot { TestBot }
}