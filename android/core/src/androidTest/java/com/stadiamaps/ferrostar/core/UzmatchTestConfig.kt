package com.stadiamaps.ferrostar.core

import uniffi.ferrostar.UzmatchConfig

/** Matching core disabled: androidTest fixtures exercise upstream behavior. */
val uzmatchDisabled =
    UzmatchConfig(
        enabled = false,
        standingSpeedThresholdMps = 0.5,
        standingDetectionPeriodMs = 7000UL,
        standingSignalExpiryMs = 5000UL,
        snapPositionStddevM = 8.0,
        snapHeadingStddevDeg = 6.0,
        snapHeadingMinSpeedMps = 4.0,
        fineAccuracyThresholdM = 25.0,
    )
