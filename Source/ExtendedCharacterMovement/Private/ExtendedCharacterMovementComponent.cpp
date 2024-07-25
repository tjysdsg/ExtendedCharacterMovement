// Copyright 2024 Mark Tang.

#include "ExtendedCharacterMovementComponent.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "Kismet/KismetMathLibrary.h"


void UExtendedCharacterMovementComponent::SetSprinting(bool Val) {
  bWantsToSprint = Val;

  // stop sliding when player sprints
  if (Val && IsCrouching()) {
    SetCrouching(false);
  }
}

bool UExtendedCharacterMovementComponent::IsSprinting() const { return bIsSprinting; }

void UExtendedCharacterMovementComponent::SetCrouching(bool Val) {
  // Disallow crouching while in air
  if (IsFalling() && Val) {
    return;
  }

  bWantsToCrouch = Val;

  if (Val) {
    SetSprinting(false);
  }
}

void UExtendedCharacterMovementComponent::ToggleSprinting() {
  SetSprinting(!bWantsToSprint);
}

void UExtendedCharacterMovementComponent::ToggleCrouching() {
  SetCrouching(!bWantsToCrouch);
}

bool UExtendedCharacterMovementComponent::IsCustomMovementMode(uint8 CustomMovementMode_) const {
  return MovementMode == EMovementMode::MOVE_Custom && CustomMovementMode == CustomMovementMode_;
}

#pragma region WallRunning

bool UExtendedCharacterMovementComponent::IsWallRunning() const {
  return IsCustomMovementMode(ECustomMovementMode::CMOVE_WallRunning);
}

bool UExtendedCharacterMovementComponent::CheckIfCanWallRun() {
  // If we're not wall running, we have to be falling in order to begin wall running
  // If we're already wall running, we don't care about status of falling
  if (!IsFalling() && !IsWallRunning())
    return false;

  // If we just jumped away from the wall, skip checking this frame so that we can have time to move away from wall
  if (bIsWallRunJumpingAway)
    return false;

  if (!FindNearbyWall(CurrentWallRunInfo)) {
    return false;
  }

  // Can't wall run if we're close to standing on a walkable floor
  FHitResult FloorHit{};
  if (IsNearWalkableFloor(FloorHit)) {
    // UE_LOG(LogTemp, Display, TEXT("Standing on floor"));
    return false;
  }

  if (!IsWallRunSurfaceAngleValid(CurrentWallRunInfo.Normal)) {
    CurrentWallRunInfo = FWallRunInfo{};
    return false;
  }

  FVector RunDirectionV = UKismetMathLibrary::ProjectVectorOnToVector(Velocity, CurrentWallRunInfo.WallRunDirection);

  // Check if looking at the wall
  FVector ForwardNoZ = UpdatedComponent->GetForwardVector();
  ForwardNoZ.Z = 0;
  ForwardNoZ.Normalize();
  FVector WallNormalNoZ = CurrentWallRunInfo.Normal;
  WallNormalNoZ.Z = 0;
  WallNormalNoZ.Normalize();
  double Cos = ForwardNoZ.Dot(WallNormalNoZ);
  // DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + WallNormalNoZ * 1000, FColor(0, 0, 255), false, 5);
  // DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + ForwardNoZ * 1000, FColor(255, 0, 0), false, 5);

  if (Cos > 0) {
    // Looking away from the wall, need to check pull away angle
    if (Cos >= UKismetMathLibrary::DegSin(WallRunPullAwayAngle)) {
      return false;
    }
  }

  // Has to be running forward fast
  if (RunDirectionV.Length() < MinWallRunSpeed) {
    return false;
  }

  return true;
}

void UExtendedCharacterMovementComponent::BeginWallRun() {
  check(bCanWallRun);
  // Set the movement mode to wall running. Unreal will handle replicating this change to all connected clients.
  SetMovementMode(EMovementMode::MOVE_Custom, ECustomMovementMode::CMOVE_WallRunning);
}

bool UExtendedCharacterMovementComponent::IsWallRunSurfaceAngleValid(const FVector &SurfaceNormal) const {
  // Return false if the surface normal is facing down
  if (SurfaceNormal.Z < -0.05f)
    return false;

  FVector NormalNoZ = FVector(SurfaceNormal.X, SurfaceNormal.Y, 0.0f);
  NormalNoZ.Normalize();

  // Find the angle between the wall and vertical plane
  double WallAngle = UKismetMathLibrary::DegAcos(FVector::DotProduct(NormalNoZ, SurfaceNormal));

  // If the wall angle is less than walkable floor angle
  // we're not wall running, we're just walking lol
  // UE_LOG(LogTemp, Display, TEXT("%f"), WallAngle);
  return WallAngle <= MaxWallRunAngle;
}

EWallRunSide UExtendedCharacterMovementComponent::CalculateWallRunSide(const FVector &SurfaceNormal) const {
  if (FVector2D::DotProduct(FVector2D(SurfaceNormal), FVector2D(GetPawnOwner()->GetActorRightVector())) > 0.0) {
    return EWallRunSide::kLeft;
  }

  return EWallRunSide::kRight;
}

FVector UExtendedCharacterMovementComponent::CalculateWallRunDirection(
    const FVector &SurfaceNormal,
    EWallRunSide Side) const {
  FVector CrossVector;
  if (Side == EWallRunSide::kLeft) {
    CrossVector = FVector(0.0f, 0.0f, 1.0f);
  } else {
    CrossVector = FVector(0.0f, 0.0f, -1.0f);
  }

  return FVector::CrossProduct(SurfaceNormal, CrossVector);
}

bool UExtendedCharacterMovementComponent::FindNearbyWall(FWallRunInfo &WallRunInfo) const {
  return FindNearbyWall(EWallRunSide::kLeft, WallRunInfo) ||
         FindNearbyWall(EWallRunSide::kRight, WallRunInfo);
}

bool UExtendedCharacterMovementComponent::FindNearbyWallFirstPass(EWallRunSide Side, FHitResult &Hit) const {
  double CharacterHalfWidth = CapsuleComponent->GetScaledCapsuleRadius();
  double CharacterHalfHeight = CapsuleComponent->GetScaledCapsuleHalfHeight();

  FVector TraceStart = UpdatedComponent->GetComponentLocation() - CharacterHalfHeight * UpdatedComponent->GetUpVector();
  FVector Delta = WallDetectionRadiusTimes * CharacterHalfWidth * UpdatedComponent->GetRightVector();
  FVector TraceEnd = TraceStart + Delta;
  if (Side == EWallRunSide::kLeft) {
    TraceEnd = TraceStart - Delta;
  }

  bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECollisionChannel::ECC_Visibility);
  return bHit && Hit.IsValidBlockingHit();
}

bool UExtendedCharacterMovementComponent::FindNearbyWall(EWallRunSide Side,
                                                         FWallRunInfo &WallRunInfo) const {
  double CharacterHalfWidth = CapsuleComponent->GetScaledCapsuleRadius();
  double CharacterHalfHeight = CapsuleComponent->GetScaledCapsuleHalfHeight();

  FHitResult FirstPassHit{};
  if (FindNearbyWallFirstPass(Side, FirstPassHit)) {
    // Second pass: check if the wall is on the expected side
    double Dot = FirstPassHit.Normal.Dot(UpdatedComponent->GetRightVector());
    if (Side == EWallRunSide::kLeft && Dot <= 0)
      return false;
    if (Side == EWallRunSide::kRight && Dot >= 0)
      return false;

    // Second pass: check if touching the wall
    FVector NormalNoZ = FirstPassHit.Normal;
    NormalNoZ.Z = 0;
    NormalNoZ.Normalize();

    FVector TraceStart =
        UpdatedComponent->GetComponentLocation() - CharacterHalfHeight * UpdatedComponent->GetUpVector();
    FVector TraceEnd = TraceStart - 1.1 * CharacterHalfWidth * NormalNoZ;

    FHitResult Hit{};
    bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECollisionChannel::ECC_Visibility);
    if (bHit && Hit.IsValidBlockingHit()) {
      DrawDebugLine(GetWorld(), TraceStart, TraceEnd, FColor(150, 150, 0), false, 5);

      WallRunInfo.Hit = Hit;
      WallRunInfo.WallRunSide = Side;
      WallRunInfo.WallRunDirection = CalculateWallRunDirection(Hit.ImpactNormal, WallRunInfo.WallRunSide).
          GetSafeNormal();
      WallRunInfo.Normal = Hit.ImpactNormal;
      WallRunInfo.WallContactLocation = Hit.ImpactPoint;

      FVector G = FVector::CrossProduct(Hit.ImpactNormal, WallRunInfo.WallRunDirection);
      if (Side == EWallRunSide::kRight)
        G = -G;
      G.Normalize();
      WallRunInfo.GravityDirection = G;

      return true;
    }
  }

  return false;
}

bool UExtendedCharacterMovementComponent::IsNearWalkableFloor(FHitResult &Hit) const {
  double CharacterHalfHeight = CapsuleComponent->GetScaledCapsuleHalfHeight();
  FVector TraceStart = UpdatedComponent->GetComponentLocation();
  FVector Delta = CharacterHalfHeight * UpdatedComponent->GetUpVector() + MinWallRunHeight;
  FVector TraceEnd = TraceStart - Delta;

  bool bBlocking = GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECollisionChannel::ECC_Visibility);
  if (bBlocking) {
    double Cos = FVector::DotProduct(FVector::UpVector, Hit.Normal);
    if (Cos <= 0)
      return false;

    return UKismetMathLibrary::DegAcos(Cos) <= GetWalkableFloorAngle();
  }

  return false;
}

void UExtendedCharacterMovementComponent::VisualWallRunInfo() const {
  FVector Contact = CurrentWallRunInfo.WallContactLocation;

  DrawDebugLine(GetWorld(), Contact, Contact + CurrentWallRunInfo.Normal * 200, FColor(0, 0, 255), false, 5);
  DrawDebugLine(GetWorld(),
                Contact,
                Contact + CurrentWallRunInfo.WallRunDirection * 200,
                FColor(0, 255, 0),
                false,
                5);
  DrawDebugLine(GetWorld(),
                Contact,
                Contact + CurrentWallRunInfo.GravityDirection * 200,
                FColor(255, 0, 0),
                false,
                5);
}

void UExtendedCharacterMovementComponent::PhysWallRunning(float DeltaTime, int32 Iterations) {
  check(bCanWallRun);

  if (DeltaTime < MIN_TICK_TIME)
    return;

  if (!CharacterOwner || (!CharacterOwner->Controller && !bRunPhysicsWithNoController && !HasAnimRootMotion() && !
                          CurrentRootMotion.HasOverrideVelocity() &&
                          CharacterOwner->GetLocalRole() != ROLE_SimulatedProxy)) {
    Acceleration = FVector::ZeroVector;
    Velocity = FVector::ZeroVector;
    return;
  }

  // Substep calculation
  float RemainingTime = DeltaTime;
  while (RemainingTime >= MIN_TICK_TIME && Iterations < MaxSimulationIterations && CharacterOwner && (
           CharacterOwner->Controller || bRunPhysicsWithNoController || HasAnimRootMotion() || CurrentRootMotion.
           HasOverrideVelocity() || CharacterOwner->GetLocalRole() == ROLE_SimulatedProxy)) {
    bJustTeleported = false;

    Iterations++;
    const float TimeTick = GetSimulationTimeStep(RemainingTime, Iterations);
    RemainingTime -= TimeTick;

    const FVector OldLocation = UpdatedComponent->GetComponentLocation();

    // Check wall run still valid
    bool bHit = CheckIfCanWallRun();
    if (!bHit) {
      SetMovementMode(MOVE_Falling);
      StartNewPhysics(RemainingTime, Iterations);
      return;
    }

    // The faster we run, the less gravity we have
    FVector Gravity = -GetGravityDirection() * GetGravityZ();
    Gravity *= UKismetMathLibrary::FClamp(1 - Velocity.Length() / GetMaxSpeed(), MinRelativeWallRunGravity, 1);
    Acceleration += Gravity;
    Acceleration = FVector::VectorPlaneProject(Acceleration, CurrentWallRunInfo.Normal);

    // Apply acceleration
    CalcVelocity(TimeTick, 0.f, false, GetMaxBrakingDeceleration());
    Velocity = FVector::VectorPlaneProject(Velocity, CurrentWallRunInfo.Normal);

    // Compute move parameters
    const FVector Delta = TimeTick * Velocity;
    FHitResult MovementHit{};
    SafeMoveUpdatedComponent(Delta, UpdatedComponent->GetComponentQuat(), true, MovementHit);
    if (MovementHit.IsValidBlockingHit()) {
      HandleImpact(MovementHit, MovementHit.Time, Delta);

      FVector ActualDelta = UpdatedComponent->GetComponentLocation() - OldLocation;
      SlideAlongSurface(Delta - ActualDelta, 1 - MovementHit.Time, MovementHit.ImpactNormal, MovementHit, true);
    }

    Velocity = (UpdatedComponent->GetComponentLocation() - OldLocation) / TimeTick;
  }
}

#pragma endregion


#pragma region Sliding

/**
 * Returns true if floor surface is valid for sliding and min speed is reached.
 * Allows sliding upwards.
 */
bool UExtendedCharacterMovementComponent::CanSlide(FHitResult &Hit) const {
  double CharacterHalfHeight = CapsuleComponent->GetScaledCapsuleHalfHeight();
  FVector TraceStart = UpdatedComponent->GetComponentLocation();
  FVector Delta = 2 * CharacterHalfHeight * UpdatedComponent->GetUpVector(); // half height of crouched capsule
  FVector TraceEnd = TraceStart - Delta;

  bool bValidSurface = GetWorld()->LineTraceSingleByChannel(Hit,
                                                            TraceStart,
                                                            TraceEnd,
                                                            ECollisionChannel::ECC_Visibility);
  bool bEnoughSpeed = Velocity.SizeSquared() > pow(MinSlideSpeed, 2);

  double Cos = FVector::DotProduct(FVector::UpVector, Hit.Normal);
  bool bValidAngle = UKismetMathLibrary::DegAcos(Cos) <= MaxSlideSurfaceAngle;

  return bValidSurface && bEnoughSpeed && bValidAngle;
}

void UExtendedCharacterMovementComponent::OnMovementModeEnterSlide() {
  bWantsToCrouch = true;

  FindFloor(UpdatedComponent->GetComponentLocation(), CurrentFloor, false, nullptr);

  UE_LOG(LogTemp, Display, TEXT("Enter slide"));
}

void UExtendedCharacterMovementComponent::OnMovementModeExitSlide() {
  // bWantsToCrouch = false;

  UE_LOG(LogTemp, Display, TEXT("Exit slide"));
}

/**
 * Based on UCharacterMovementComponent::PhysWalking
 */
void UExtendedCharacterMovementComponent::PhysSliding(float deltaTime, int32 Iterations) {
  if (deltaTime < MIN_TICK_TIME) {
    return;
  }

  if (!CharacterOwner || (!CharacterOwner->Controller && !bRunPhysicsWithNoController && !HasAnimRootMotion() && !
                          CurrentRootMotion.HasOverrideVelocity() &&
                          CharacterOwner->GetLocalRole() != ROLE_SimulatedProxy)) {
    Acceleration = FVector::ZeroVector;
    Velocity = FVector::ZeroVector;
    return;
  }

  if (!UpdatedComponent->IsQueryCollisionEnabled()) {
    SetMovementMode(MOVE_Walking);
    return;
  }

  FHitResult SlideSurfaceHit;
  if (!CanSlide(SlideSurfaceHit)) {
    SetMovementMode(MOVE_Walking);
    StartNewPhysics(deltaTime, Iterations);
    return;
  }

  bJustTeleported = false;
  bool bCheckedFall = false;
  bool bTriedLedgeMove = false;
  float remainingTime = deltaTime;

  const EMovementMode StartingMovementMode = MovementMode;
  const uint8 StartingCustomMovementMode = CustomMovementMode;

  // Perform the move
  while (remainingTime >= MIN_TICK_TIME && Iterations < MaxSimulationIterations && CharacterOwner && (
           CharacterOwner->Controller || bRunPhysicsWithNoController || HasAnimRootMotion() || CurrentRootMotion.
           HasOverrideVelocity() || CharacterOwner->GetLocalRole() == ROLE_SimulatedProxy)) {
    Iterations++;
    bJustTeleported = false;
    const float timeTick = GetSimulationTimeStep(remainingTime, Iterations);
    remainingTime -= timeTick;

    // Save current values
    UPrimitiveComponent *const OldBase = GetMovementBase();
    const FVector PreviousBaseLocation = (OldBase != NULL) ? OldBase->GetComponentLocation() : FVector::ZeroVector;
    const FVector OldLocation = UpdatedComponent->GetComponentLocation();
    const FFindFloorResult OldFloor = CurrentFloor;

    RestorePreAdditiveRootMotionVelocity();

    // Ensure velocity is horizontal.
    MaintainHorizontalGroundVelocity();
    const FVector OldVelocity = Velocity;

    // Prevent acceleration up along a slope, only allows steering left/right or braking
    FVector SlopeUpwardDirection = FVector::VectorPlaneProject(-GetGravityDirection(), CurrentFloor.HitResult.Normal);
    Acceleration -= Acceleration.ProjectOnTo(SlopeUpwardDirection);

    // Add sliding gravity
    FVector SlideAccelDirection = FVector::VectorPlaneProject(
        GetGravityDirection(),
        CurrentFloor.HitResult.Normal).GetSafeNormal();
    Acceleration += SlideAccelDirection * SlideAcceleration;

    // Apply acceleration
    CalcVelocity(timeTick, GroundFriction * SlideFrictionFactor, false, GetMaxBrakingDeceleration());

    // Compute move parameters
    const FVector MoveVelocity = Velocity;
    const FVector Delta = timeTick * MoveVelocity;
    const bool bZeroDelta = Delta.IsNearlyZero();
    FStepDownResult StepDownResult;

    if (bZeroDelta) {
      remainingTime = 0.f;
    } else {
      // try to move forward
      MoveAlongFloor(MoveVelocity, timeTick, &StepDownResult);

      if (IsSwimming()) //just entered water
      {
        StartSwimming(OldLocation, OldVelocity, timeTick, remainingTime, Iterations);
        return;
      } else if (MovementMode != StartingMovementMode || CustomMovementMode != StartingCustomMovementMode) {
        // pawn ended up in a different mode, probably due to the step-up-and-over flow
        // let's refund the estimated unused time (if any) and keep moving in the new mode
        const float DesiredDist = Delta.Size();
        if (DesiredDist > UE_KINDA_SMALL_NUMBER) {
          const float ActualDist = (UpdatedComponent->GetComponentLocation() - OldLocation).Size2D();
          remainingTime += timeTick * (1.f - FMath::Min(1.f, ActualDist / DesiredDist));
        }
        StartNewPhysics(remainingTime, Iterations);
        return;
      }
    }

    // Update floor.
    // StepUp might have already done it for us.
    if (StepDownResult.bComputedFloor) {
      CurrentFloor = StepDownResult.FloorResult;
    } else {
      FindFloor(UpdatedComponent->GetComponentLocation(), CurrentFloor, bZeroDelta, NULL);
    }

    // check for ledges here
    const bool bCheckLedges = !CanWalkOffLedges();
    if (bCheckLedges && !CurrentFloor.IsWalkableFloor()) {
      // calculate possible alternate movement
      const FVector GravDir = GetGravityDirection();
      const FVector NewDelta = bTriedLedgeMove ? FVector::ZeroVector : GetLedgeMove(OldLocation, Delta, GravDir);
      if (!NewDelta.IsZero()) {
        // first revert this move
        RevertMove(OldLocation, OldBase, PreviousBaseLocation, OldFloor, false);

        // avoid repeated ledge moves if the first one fails
        bTriedLedgeMove = true;

        // Try new movement direction
        Velocity = NewDelta / timeTick;
        remainingTime += timeTick;
        continue;
      } else {
        // see if it is OK to jump
        bool bMustJump = bZeroDelta || (OldBase == NULL || (
                                          !OldBase->IsQueryCollisionEnabled() && MovementBaseUtility::IsDynamicBase(
                                              OldBase)));
        if ((bMustJump || !bCheckedFall) && CheckFall(OldFloor,
                                                      CurrentFloor.HitResult,
                                                      Delta,
                                                      OldLocation,
                                                      remainingTime,
                                                      timeTick,
                                                      Iterations,
                                                      bMustJump)) {
          return;
        }
        bCheckedFall = true;

        // revert this move
        RevertMove(OldLocation, OldBase, PreviousBaseLocation, OldFloor, true);
        remainingTime = 0.f;
        break;
      }
    } else {
      // Validate the floor check
      if (CurrentFloor.IsWalkableFloor()) {
        if (ShouldCatchAir(OldFloor, CurrentFloor)) {
          HandleWalkingOffLedge(OldFloor.HitResult.ImpactNormal, OldFloor.HitResult.Normal, OldLocation, timeTick);
          if (IsMovingOnGround()) {
            // If still walking, then fall. If not, assume the user set a different mode they want to keep.
            StartFalling(Iterations, remainingTime, timeTick, Delta, OldLocation);
          }
          return;
        }

        AdjustFloorHeight();
        SetBase(CurrentFloor.HitResult.Component.Get(), CurrentFloor.HitResult.BoneName);
      } else if (CurrentFloor.HitResult.bStartPenetrating && remainingTime <= 0.f) {
        // The floor check failed because it started in penetration
        // We do not want to try to move downward because the downward sweep failed, rather we'd like to try to pop out of the floor.
        FHitResult Hit(CurrentFloor.HitResult);
        Hit.TraceEnd = Hit.TraceStart + RotateGravityToWorld(FVector(0.f, 0.f, MAX_FLOOR_DIST));
        const FVector RequestedAdjustment = GetPenetrationAdjustment(Hit);
        ResolvePenetration(RequestedAdjustment, Hit, UpdatedComponent->GetComponentQuat());
        bForceNextFloorCheck = true;
      }

      // check if just entered water
      if (IsSwimming()) {
        StartSwimming(OldLocation, Velocity, timeTick, remainingTime, Iterations);
        return;
      }

      // See if we need to start falling.
      if (!CurrentFloor.IsWalkableFloor() && !CurrentFloor.HitResult.bStartPenetrating) {
        const bool bMustJump = bJustTeleported || bZeroDelta || (
                                 OldBase == NULL || (
                                   !OldBase->IsQueryCollisionEnabled() && MovementBaseUtility::IsDynamicBase(OldBase)));
        if ((bMustJump || !bCheckedFall) && CheckFall(OldFloor,
                                                      CurrentFloor.HitResult,
                                                      Delta,
                                                      OldLocation,
                                                      remainingTime,
                                                      timeTick,
                                                      Iterations,
                                                      bMustJump)) {
          return;
        }
        bCheckedFall = true;
      }
    }

    // Allow overlap events and such to change physics state and velocity
    if (IsMovingOnGround()) {
      // Make velocity reflect actual move
      if (!bJustTeleported && !HasAnimRootMotion() && !CurrentRootMotion.HasOverrideVelocity() && timeTick >=
          MIN_TICK_TIME) {
        Velocity = (UpdatedComponent->GetComponentLocation() - OldLocation) / timeTick;
        MaintainHorizontalGroundVelocity();
      }
    }

    // If we didn't move at all this iteration then abort (since future iterations will also be stuck).
    if (UpdatedComponent->GetComponentLocation() == OldLocation) {
      remainingTime = 0.f;
      break;
    }
  }

  if (IsMovingOnGround()) {
    MaintainHorizontalGroundVelocity();
  }
}

#pragma endregion

UExtendedCharacterMovementComponent::UExtendedCharacterMovementComponent() {
  NavAgentProps.bCanCrouch = true;

  AirControl = 1.0f;

  bCanWalkOffLedges = true;
  bCanWalkOffLedgesWhenCrouching = true;
}

void UExtendedCharacterMovementComponent::BeginPlay() {
  Super::BeginPlay();

  CapsuleComponent = GetPawnOwner()->FindComponentByClass<UCapsuleComponent>();
  check(CapsuleComponent);

  if (MaxSlideSurfaceAngle + MaxWallRunAngle > 90) {
    UE_LOG(LogTemp,
           Error,
           TEXT("(MaxSlideSurfaceAngle + MaxWallRunAngle) must be <=90 degrees. "
             "Otherwise certain surfaces can be valid for both wallrunning and sliding."));
  }

  OnMovementModeChanged(MOVE_None, CMOVE_MAX);
}

void UExtendedCharacterMovementComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction) {
  // check if we can sprint
  if (bWantsToSprint) {
    FVector Velocity2D = GetPawnOwner()->GetVelocity();
    FVector Forward2D = GetPawnOwner()->GetActorForwardVector();
    Velocity2D.Z = 0.0f;
    Forward2D.Z = 0.0f;
    Velocity2D.Normalize();
    Forward2D.Normalize();

    if (Velocity2D.Size() > SMALL_NUMBER) {
      // Only set bIsSprinting to true if the player is moving forward and not falling
      bIsSprinting = !IsFalling()
                     && FVector::DotProduct(Velocity2D, Forward2D) > 0.5f;
    }

    // Automatically stop sprinting if not moving
    else {
      bIsSprinting = false;
      SetSprinting(false);
    }

  } else {
    bIsSprinting = false;
  }

  // sprint status changed
  if (bWasSprinting != bIsSprinting) {
    OnSprintStatusChangedCallbacks.Broadcast(bIsSprinting);
  }
  bWasSprinting = bIsSprinting;

  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UExtendedCharacterMovementComponent::OnMovementModeChanged(
    EMovementMode PreviousMovementMode,
    uint8 PreviousCustomMode) {
  Super::OnMovementModeChanged(PreviousMovementMode, PreviousCustomMode);

  // Slide
  if (PreviousMovementMode == MOVE_Custom && PreviousCustomMode == CMOVE_Sliding)
    OnMovementModeExitSlide();
  if (IsCustomMovementMode(CMOVE_Sliding))
    OnMovementModeEnterSlide();

  if (MovementMode != MOVE_Falling) {
    // Unlikely but we want to be sure
    bIsWallRunJumpingAway = false;
  }

  OnMovementModeChangedCallbacks.Broadcast(MovementMode, static_cast<ECustomMovementMode>(CustomMovementMode));
}

void UExtendedCharacterMovementComponent::UpdateCharacterStateBeforeMovement(float DeltaSeconds) {
  // Slide
  // Start sliding when user crouches
  if (MovementMode == MOVE_Walking && bWantsToCrouch) {
    FHitResult Hit;
    if (CanSlide(Hit)) {
      SetMovementMode(MOVE_Custom, CMOVE_Sliding);
    }

    // Stop sliding when user uncrouches
  } else if (IsCustomMovementMode(CMOVE_Sliding) && !bWantsToCrouch) {
    SetMovementMode(MOVE_Walking);
  }

  // Check wall run
  else if (bCanWallRun && CheckIfCanWallRun()) {
    // VisualWallRunInfo();
    BeginWallRun();
  }

  Super::UpdateCharacterStateBeforeMovement(DeltaSeconds);
}

void UExtendedCharacterMovementComponent::PhysCustom(float DeltaTime, int32 Iterations) {
  // Phys* functions should only run for characters with ROLE_Authority or ROLE_AutonomousProxy. However, Unreal calls PhysCustom in
  // two seperate locations, one of which doesn't check the role, so we must check it here to prevent this code from running on simulated proxies.
  if (GetOwner()->GetLocalRole() == ROLE_SimulatedProxy)
    return;

  switch (CustomMovementMode) {
  case ECustomMovementMode::CMOVE_WallRunning:
    PhysWallRunning(DeltaTime, Iterations);
    break;
  case ECustomMovementMode::CMOVE_Sliding:
    PhysSliding(DeltaTime, Iterations);
    break;
  default:
    break;
  }
}

void UExtendedCharacterMovementComponent::PhysFalling(float DeltaTime, int32 Iterations) {
  // Reset wall run jumping status if we're sufficiently away from the wall
  // Otherwise we prevent any acceleration towards the wall
  // Meanwhile, acceleration away from the wall is preserved
  if (bIsWallRunJumpingAway) {
    FHitResult Hit{};
    if (FindNearbyWallFirstPass(CurrentWallRunInfo.WallRunSide, Hit)) {
      double Cos = FVector::DotProduct(Acceleration, CurrentWallRunInfo.Normal);
      if (Cos < 0) {
        Acceleration = UKismetMathLibrary::ProjectVectorOnToPlane(Acceleration, CurrentWallRunInfo.Normal);
      }
    } else {
      bIsWallRunJumpingAway = false;
    }
  }

  Super::PhysFalling(DeltaTime, Iterations);
}

bool UExtendedCharacterMovementComponent::IsMovingOnGround() const {
  return Super::IsMovingOnGround() || IsCustomMovementMode(CMOVE_Sliding);
}

float UExtendedCharacterMovementComponent::GetMaxSpeed() const {
  if (bIsSprinting && !IsCrouching()) {
    return SprintSpeed;
  }

  if (MovementMode != MOVE_Custom)
    return Super::GetMaxSpeed();

  switch (CustomMovementMode) {
  case CMOVE_WallRunning:
    return MaxWallRunSpeed;
  case CMOVE_Sliding:
    return MaxSlideSpeed;
  default:
    UE_LOG(LogTemp, Fatal, TEXT("Invalid Movement Mode"))
    check(false);
    return 0;
  }
}

float UExtendedCharacterMovementComponent::GetMaxAcceleration() const {
  if (IsMovingOnGround() && bIsSprinting)
    return SprintAcceleration;

  return Super::GetMaxAcceleration();
}

float UExtendedCharacterMovementComponent::GetMaxBrakingDeceleration() const {
  if (MovementMode != MOVE_Custom)
    return Super::GetMaxBrakingDeceleration();

  switch (CustomMovementMode) {
  case CMOVE_Sliding:
    return BrakingDecelerationSliding;
  case CMOVE_WallRunning:
    return 0.f;
  default:
    UE_LOG(LogTemp, Fatal, TEXT("Invalid Movement Mode"))
    check(false);
    return -1.f;
  }
}

bool UExtendedCharacterMovementComponent::CanAttemptJump() const {
  return Super::CanAttemptJump() || IsWallRunning() || IsCustomMovementMode(CMOVE_Sliding);
}

bool UExtendedCharacterMovementComponent::DoJump(bool bReplayingMoves) {
  bool bIsWallRunning = IsWallRunning(); // IMPORTANT: DoJump will make this always false
  bool bIsSliding = IsCustomMovementMode(CMOVE_Sliding);

  if (bIsSliding)
    UE_LOG(LogTemp, Display, TEXT("Jump while sliding"));

  if (Super::DoJump(bReplayingMoves)) {
    if (bIsWallRunning) {
      bIsWallRunJumpingAway = true;
      Velocity += CurrentWallRunInfo.Normal * WallRunJumpAwayVelocity;
    }

    return true;
  }

  return false;
}