package com.team2898.engine.async.pools

import kotlinx.coroutines.newFixedThreadPoolContext

val IOPool = newFixedThreadPoolContext(1, "IO Pool")
val ComputePool = newFixedThreadPoolContext(1, "Compute Pool")
