package com.team2898.engine.test.test

/*
import arrow.core.right
import com.github.sh0nk.matplotlib4j.Plot
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.row
import com.team2898.robot.subsystem.Drivetrain
import io.kotlintest.specs.StringSpec


class FollowerTest : StringSpec() {

//    val ref = Matrix(Matrix(arrayOf(row(path[currentIndex]., path[currentIndex].y))).T.data)

    // pathfinder gives us: x ref, y ref, +v ref, heading ref
    // x ref, y ref, v ref, w ref

    init {
        val path = points
        val sb = StringBuilder()
        val size = path.size


//        path.apply { forEach { println(it) } }

//        b=0.18 z=0.7

        val ramsete_ref = ramsete(
                positionReference = RigidTransform2d(Translation2d(1.0, 0.0), Rotation2d(1.0, 0.0)),
                velocityReference = Twist2d(1.0, 0.0, 0.0),
                pose = RigidTransform2d(Translation2d(0.0, 0.0), Rotation2d(1.0, 0.0)),
                b = 0.5,
                zeta = 0.5
        )


        var currentTime = 0.0
        var currentIndex = 0

        val X = mutableListOf<Double>()
        val Y = mutableListOf<Double>()

        //Drivetrain.state = Drivetrain.state.copy(pose = Drivetrain.state.pose.transformBy(RigidTransform2d(Translation2d(0.0, 100.0), Rotation2d(1.0, 0.0))))
        while (size > currentIndex) {
            val target = path[currentIndex]
            val ramsete_ref = ramsete(
                    positionReference = RigidTransform2d(Translation2d(target.x, target.y), Rotation2d.createFromRadians(target.heading)),
                    velocityReference = Twist2d(target.vel, 0.0, target.angVel),
                    pose = Drivetrain.state.pose,
//                    b = 0.18,
//                    zeta = 0.7
                    b = 0.6,
                    zeta = 0.9
            )
            val ref = Matrix(Matrix(arrayOf(row(ramsete_ref.first, ramsete_ref.second))).T.data)
            Drivetrain.step(Drivetrain.genU(ref))

            currentTime += Drivetrain.dt

            if (currentTime > target.t) currentIndex++
            //println("${target.vel} ${target.angVel} ${ramsete_ref.first/2 + ramsete_ref.second/2} ${-ramsete_ref.first/Drivetrain.wheelbase + ramsete_ref.second/Drivetrain.wheelbase}")
            //println("ramsete ref: ${ramsete_ref.first}, dt: ${Drivetrain.state.vel.dx - Drivetrain.state.vel.dtheta * Drivetrain.wheelbase / 2}")
            println("x: ${target.x}, y: ${target.y}, dtx: ${Drivetrain.state.pose.x}, dty: ${Drivetrain.state.pose.y}, dtv: ${Drivetrain.state.vel.dx}")
            X.add(Drivetrain.state.pose.x)
            Y.add(Drivetrain.state.pose.y)
        }

        Plot.create().apply {
            plot().add(X, Y).add(path.map { it.x }, path.map { it.y })
            xlabel("X")
            ylabel("Y")
            title("Drivetrain")
            legend()
            show()
        }
        //File("test.csv").writeText(sb.toString())
    }
}

*/