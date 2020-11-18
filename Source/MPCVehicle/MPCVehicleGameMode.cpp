// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "MPCVehicleGameMode.h"
#include "MPCVehiclePawn.h"
#include "MPCVehicleHud.h"

AMPCVehicleGameMode::AMPCVehicleGameMode()
{
	DefaultPawnClass = AMPCVehiclePawn::StaticClass();
	HUDClass = AMPCVehicleHud::StaticClass();
}
