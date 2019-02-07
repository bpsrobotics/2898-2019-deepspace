package com.team2898.robot.motion.path

import com.team2898.engine.motion.pathfinder.ProfileSettings
import jaci.pathfinder.Trajectory

val testAuto = ProfileSettings(
        hz = 100,
        maxVel = 10.0,
        maxAcc = 7.00,
        maxJerk = 10.0,
        wheelbaseWidth = 2.0,
        wayPoints = testPath,
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        modified = false,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)