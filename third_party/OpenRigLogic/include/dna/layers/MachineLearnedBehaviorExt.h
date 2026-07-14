// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <cstdint>

namespace dna {

enum class MachineLearnedBehaviorOperationType : std::uint16_t {
    Unspecified,
    Gather,
    Scatter,
    MLP,
    WeightedSum
};

enum class MachineLearnedBehaviorParameterKey : std::uint16_t {
    JointTranslationType,
    JointRotationType,
    JointScaleType,
    JointCoordinateSystemAxisX,
    JointCoordinateSystemAxisY,
    JointCoordinateSystemAxisZ,
    JointRotationSignAxisX,
    JointRotationSignAxisY,
    JointRotationSignAxisZ,
    JointRotationSequence
};

}  // namespace dna
