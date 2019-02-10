package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.math.linear.get
import com.team2898.engine.motion.pathfinder.TrajPoint
import com.team2898.engine.motion.pathfinder.TrajectoryGenerator
import com.team2898.robot.commands.PathFollower
import com.team2898.robot.motion.path.testAuto
import com.team2898.robot.motion.pathfinder.ProfileGenerator
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotBase.startRobot
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import jaci.pathfinder.Trajectory
import kotlinx.serialization.ImplicitReflectionSerializer

@ImplicitReflectionSerializer
object Robot : TimedRobot() {
    override fun robotInit() {
        AsyncLooper(50.0) {
            SmartDashboard.putNumber("left vel", Drivetrain.x[0, 0])
            SmartDashboard.putNumber("right vel", Drivetrain.x[1, 0])
            SmartDashboard.putNumber("left pos", Drivetrain.leftEnc.distance)
            SmartDashboard.putNumber("right pos", Drivetrain.rightEnc.distance)
            SmartDashboard.putNumber("kalman left", Drivetrain.xHat[0, 0])
            SmartDashboard.putNumber("kalman right", Drivetrain.xHat[1, 0])
        }.start()
    }
    val trajPoint: Array<TrajPoint>
    init {
        val path = ProfileGenerator.genUnmodifiedProfile(testAuto)
        trajPoint = TrajectoryGenerator().getTrajPoints(path)
    }

    val autoCommand = PathFollower(trajPoint)

    override fun autonomousInit() {
        autoCommand.start()
    }
}

@ImplicitReflectionSerializer
fun main() {
    RobotBase.startRobot { Robot }
}