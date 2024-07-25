// Copyright 2024 Mark Tang.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "ExtendedCharacterMovementComponent.generated.h"


UENUM(BlueprintType)
enum ECustomMovementMode : uint8 {
  CMOVE_None UMETA(DisplayName = "None"),
  CMOVE_WallRunning UMETA(DisplayName = "WallRunning"),
  CMOVE_Sliding UMETA(DisplayName = "Sliding"),
  CMOVE_MAX UMETA(Hidden),
};


UENUM(BlueprintType)
enum class EWallRunSide : uint8 {
  kLeft = 0 UMETA(DisplayName = "Left"),
  kRight = 1 UMETA(DisplayName = "Right"),
};


USTRUCT()
struct FWallRunInfo {
  GENERATED_BODY()

  FHitResult Hit{};

  // The side of the wall the player is running on.
  EWallRunSide WallRunSide = EWallRunSide::kLeft;

  // The direction the character is currently wall running in
  FVector WallRunDirection = FVector::ZeroVector;
  FVector WallContactLocation = FVector::ZeroVector;

  FVector Normal = FVector::ZeroVector;
  FVector GravityDirection = FVector::ZeroVector;
};


DECLARE_MULTICAST_DELEGATE_TwoParams(
    FMovementModeDelegate,
    EMovementMode /* MovementMode */,
    ECustomMovementMode /* CustomMovementMode */);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnSprintChangedDelegate, bool /* IsSprinting */);


UCLASS(ClassGroup=(Game), meta=(BlueprintSpawnableComponent))
class EXTENDEDCHARACTERMOVEMENT_API UExtendedCharacterMovementComponent : public UCharacterMovementComponent {
  GENERATED_BODY()

public:
  FMovementModeDelegate OnMovementModeChangedCallbacks;
  FOnSprintChangedDelegate OnSprintStatusChangedCallbacks;

#pragma region Sprint

  UPROPERTY(EditAnywhere,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sprint")
  float SprintSpeed = 1000.0f;

  UPROPERTY(EditAnywhere,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sprint")
  float SprintAcceleration = 2000.0f;

#pragma endregion

#pragma region WallRunning

  UPROPERTY(EditAnywhere,
    BlueprintReadWrite,
    Category = "Extended Character Movement|Wall Running")
  bool bCanWallRun = true;

  // When player is looking away from the wall, this is the max angle between wall and player look direction
  // within which wall can continue.
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running",
    meta=(ClampMin=0, ClampMax=90)
  )
  float WallRunPullAwayAngle = 45.0f;

  // Min angle between the wall and the vertical plane a wall must have in order for it to be runnable.
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running",
    meta=(ClampMin="0.0", ClampMax="90.0", ForceUnits="degrees"))
  float MaxWallRunAngle = 20.f;

  // The max distance between the capsule and a wall for the wall to be detected as close by,
  // represented as the number of times the capsule radius.
  // This is also the cosine of the player look direction and wall tangent, both projected onto the horizontal plane.
  // Note that a wall being close by doesn't mean it's valid for wall running.
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running")
  float WallDetectionRadiusTimes = 4.0f;

  // The minimum gravity to apply to the character while wall running. Expressed as the portion of *g*.
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running",
    meta=(ClampMin=0, ClampMax=1))
  float MinRelativeWallRunGravity = 0.1f;

  // Wall Run
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running")
  float MinWallRunSpeed = 200.f;

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running")
  float MaxWallRunSpeed = 600.f;

  /// The minimum height above ground to be able to wall run
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running")
  float MinWallRunHeight = 30.f;

  /// The instant velocity applied to the character if jumping while wall running.
  /// The direction of the jump is same as the normal vector of the wall.
  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Wall Running")
  float WallRunJumpAwayVelocity = 300.f;

#pragma endregion

#pragma region Slide

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sliding")
  float MinSlideSpeed = 400.f;

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sliding")
  float MaxSlideSpeed = 2000.f;

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sliding")
  float SlideAcceleration = 1000.f;

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sliding")
  float SlideFrictionFactor = .02f;

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sliding")
  float BrakingDecelerationSliding = 1000.f;

  UPROPERTY(EditDefaultsOnly,
    BlueprintReadOnly,
    Category = "Extended Character Movement|Sliding")
  float MaxSlideSurfaceAngle = 60.f;

#pragma endregion

#pragma region APIs

public:
  UFUNCTION(BlueprintCallable)
  void SetSprinting(bool Val);

  UFUNCTION(BlueprintCallable)
  bool IsSprinting() const;

  UFUNCTION(BlueprintCallable)
  void SetCrouching(bool Val);

  UFUNCTION(BlueprintCallable)
  void ToggleSprinting();

  UFUNCTION(BlueprintCallable)
  void ToggleCrouching();

  // Returns true if the movement mode is custom and matches the provided custom movement mode
  bool IsCustomMovementMode(uint8 CustomMovementMode) const;

#pragma endregion

#pragma region WallRunning

  bool IsWallRunning() const;

  UFUNCTION(BlueprintCallable)
  void BeginWallRun();

  void PhysWallRunning(float DeltaTime, int32 Iterations);

  bool IsWallRunSurfaceAngleValid(const FVector &SurfaceNormal) const;

  EWallRunSide CalculateWallRunSide(const FVector &SurfaceNormal) const;
  FVector CalculateWallRunDirection(const FVector &SurfaceNormal, EWallRunSide Side) const;

  bool FindNearbyWall(FWallRunInfo &WallRunInfo) const;
  bool FindNearbyWall(EWallRunSide Side, FWallRunInfo &WallRunInfo) const;
  bool IsNearWalkableFloor(FHitResult &Hit) const;

  bool CheckIfCanWallRun();

#pragma endregion

#pragma region Sliding

  bool CanSlide(FHitResult &Hit) const;

  void OnMovementModeEnterSlide();

  void OnMovementModeExitSlide();

  void PhysSliding(float DeltaTime, int32 Iterations);

#pragma endregion

#pragma region Overrides

protected:
  UExtendedCharacterMovementComponent();
  void BeginPlay() override;

public:
  void TickComponent(float DeltaTime,
                     ELevelTick TickType,
                     FActorComponentTickFunction *ThisTickFunction) override;

  void OnMovementModeChanged(EMovementMode PreviousMovementMode, uint8 PreviousCustomMode) override;
  void UpdateCharacterStateBeforeMovement(float DeltaSeconds) override;
  void PhysCustom(float DeltaTime, int32 Iterations) override;
  void PhysFalling(float DeltaTime, int32 Iterations) override;

  bool IsMovingOnGround() const override;

  float GetMaxSpeed() const override;
  float GetMaxAcceleration() const override;
  float GetMaxBrakingDeceleration() const override;
  bool CanAttemptJump() const override;
  bool DoJump(bool bReplayingMoves) override;

#pragma endregion

  void VisualWallRunInfo() const;

  /// Find if a wall is nearby
  bool FindNearbyWallFirstPass(EWallRunSide Side, FHitResult &Hit) const;

private:
  // Wants to sprint but maybe we cannot
  bool bWantsToSprint : 1 = false;
  // Wants to sprint and yes we can
  bool bIsSprinting : 1 = false;
  // If sprinting last frame
  bool bWasSprinting : 1 = false;

  bool bIsWallRunJumpingAway : 1 = false;
  FWallRunInfo CurrentWallRunInfo{};

  UPROPERTY()
  UCapsuleComponent *CapsuleComponent = nullptr;
};