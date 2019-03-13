package com.team2898.robot.commands

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.controlLoops.modernControl.ramsete
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.get
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.pathfinder.TrajPoint
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import jaci.pathfinder.Trajectory


class PathFollower(val path: Pair<Trajectory, Trajectory>) : Command() {

    val size = path.first.length()

    var currentIndex = 0

    override fun isFinished(): Boolean = currentIndex == size
    override fun execute() {
        println("exec")
        val target = Pair(path.first[currentIndex], path.second[currentIndex])
        val r = Matrix(Matrix(arrayOf(row(target.first.position, target.first.velocity, target.second.position, target.second.velocity))).T.data)
        print(r)
//        val u = Drivetrain.genU(r,x= Drivetrain.x)
//        Drivetrain.openLoopPower(DriveSignal(u[0, 0], u[1, 0]))
        currentIndex ++
    }

    override fun end() {
        println("ended")
    }

    override fun initialize() {
        Drivetrain.encoders { reset() }
        currentIndex = 0
        println(size)
        println(path.first.length())
    }
}
