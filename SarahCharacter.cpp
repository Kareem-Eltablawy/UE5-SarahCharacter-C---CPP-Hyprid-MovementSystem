#include "Sarah/SarahCharacter.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Controller.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "DrawDebugHelpers.h"
#include "Misc/AssertionMacros.h"

// Memory Pool Implementation with error handling
bool ASarahCharacter::CreateMemoryPool(FMemoryPool** OutPool, size_t block_size, size_t block_count)
{
    if (!OutPool || block_size == 0 || block_count == 0)
    {
        LogError(TEXT("CreateMemoryPool: Invalid parameters"));
        return false;
    }

    PerformanceCounter start_cycles = FPlatformTime::Cycles64();

    // Allocate pool management structure
    FMemoryPool* pool = (FMemoryPool*)FMemory::Malloc(sizeof(FMemoryPool));
    if (!pool)
    {
        LogError(TEXT("CreateMemoryPool: Failed to allocate pool structure"));
        return false;
    }

    // Initialize pool structure
    FMemory::Memzero(pool, sizeof(FMemoryPool));
    pool->block_size = block_size;
    pool->block_count = block_count;
    pool->free_count = block_count;

    // Allocate contiguous memory block for the pool
    pool->memory = FMemory::Malloc(block_size * block_count);
    if (!pool->memory)
    {
        LogError(TEXT("CreateMemoryPool: Failed to allocate pool memory"));
        FMemory::Free(pool);
        return false;
    }

    // Allocate free list
    pool->free_list = (void**)FMemory::Malloc(sizeof(void*) * block_count);
    if (!pool->free_list)
    {
        LogError(TEXT("CreateMemoryPool: Failed to allocate free list"));
        FMemory::Free(pool->memory);
        FMemory::Free(pool);
        return false;
    }

    // Initialize free list with pointers to each block
    for (size_t i = 0; i < block_count; i++)
    {
        pool->free_list[i] = (char*)pool->memory + (i * block_size);
    }

    PerformanceMetrics.memory_allocation_cycles += FPlatformTime::Cycles64() - start_cycles;
    *OutPool = pool;
    return true;
}

void ASarahCharacter::DestroyMemoryPool(FMemoryPool* pool)
{
    if (pool)
    {
        if (pool->memory) FMemory::Free(pool->memory);
        if (pool->free_list) FMemory::Free(pool->free_list);
        FMemory::Free(pool);
    }
}

void* ASarahCharacter::PoolAlloc(FMemoryPool* pool)
{
    if (!pool)
    {
        LogError(TEXT("PoolAlloc: Null pool"));
        return nullptr;
    }

    // Return nullptr if pool is exhausted
    if (pool->free_count == 0)
    {
        PerformanceMetrics.memory_pool_misses++;
        LogError(TEXT("PoolAlloc: Pool exhausted"));
        return nullptr;
    }

    void* block = pool->free_list[--pool->free_count];
    pool->allocation_count++;
    PerformanceMetrics.memory_pool_hits++;
    return block;
}

bool ASarahCharacter::PoolFree(FMemoryPool* pool, void* block)
{
    if (!pool || !block)
    {
        LogError(TEXT("PoolFree: Invalid parameters"));
        return false;
    }

    // Validate block is within pool memory range
    uintptr_t block_addr = (uintptr_t)block;
    uintptr_t pool_start = (uintptr_t)pool->memory;
    uintptr_t pool_end = pool_start + (pool->block_size * pool->block_count);

    if (block_addr < pool_start || block_addr >= pool_end)
    {
        LogError(TEXT("PoolFree: Block not in pool memory range"));
        return false;
    }

    // Check if free list is full
    if (pool->free_count >= pool->block_count)
    {
        LogError(TEXT("PoolFree: Free list full"));
        return false;
    }

    pool->free_list[pool->free_count++] = block;
    pool->free_count_total++;
    return true;
}

// Stack Arena Implementation with error handling
bool ASarahCharacter::CreateStackArena(FStackArena** OutArena, size_t size)
{
    if (!OutArena || size == 0)
    {
        LogError(TEXT("CreateStackArena: Invalid parameters"));
        return false;
    }

    PerformanceCounter start_cycles = FPlatformTime::Cycles64();

    FStackArena* arena = (FStackArena*)FMemory::Malloc(sizeof(FStackArena));
    if (!arena)
    {
        LogError(TEXT("CreateStackArena: Failed to allocate arena structure"));
        return false;
    }

    FMemory::Memzero(arena, sizeof(FStackArena));
    arena->size = size;

    arena->memory = (uint8_t*)FMemory::Malloc(size);
    if (!arena->memory)
    {
        LogError(TEXT("CreateStackArena: Failed to allocate arena memory"));
        FMemory::Free(arena);
        return false;
    }

    PerformanceMetrics.memory_allocation_cycles += FPlatformTime::Cycles64() - start_cycles;
    *OutArena = arena;
    return true;
}

void ASarahCharacter::DestroyStackArena(FStackArena* arena)
{
    if (arena)
    {
        if (arena->memory) FMemory::Free(arena->memory);
        FMemory::Free(arena);
    }
}

void* ASarahCharacter::ArenaAlloc(FStackArena* arena, size_t size)
{
    if (!arena || !arena->memory)
    {
        LogError(TEXT("ArenaAlloc: Invalid arena"));
        return nullptr;
    }

    // Align to 16-byte boundary for SIMD compatibility
    size = (size + 15) & ~15;
    size_t old_offset = arena->offset;
    size_t new_offset = old_offset + size;

    // Return nullptr if arena doesn't have enough space
    if (new_offset > arena->size)
    {
        PerformanceMetrics.stack_arena_overflows++;
        LogError(TEXT("ArenaAlloc: Stack arena overflow"));
        return nullptr;
    }

    arena->offset = new_offset;

    // Track peak usage
    if (new_offset > arena->peak_usage)
    {
        arena->peak_usage = new_offset;
    }

    return arena->memory + old_offset;
}

void ASarahCharacter::ArenaResetFrame(FStackArena* arena, FrameID frame_id)
{
    if (arena && arena->frame_id != frame_id)
    {
        arena->offset = 0;
        arena->frame_id = frame_id;
    }
}

// Camera Rotation Measurement System with error handling
bool ASarahCharacter::StartCameraRotationMeasurement()
{
    if (!FollowCamera)
    {
        LogError(TEXT("StartCameraRotationMeasurement: No follow camera"));
        return false;
    }

    if (!bCameraMeasurementActive)
    {
        bCameraMeasurementActive = true;
        bCameraMeasurementInitialized = false;
        CameraYawRotation = 0.0f;
        return true;
    }
    return false;
}

void ASarahCharacter::StopCameraRotationMeasurement()
{
    bCameraMeasurementActive = false;
}

bool ASarahCharacter::UpdateCameraRotationMeasurement()
{
    if (!bCameraMeasurementActive || !FollowCamera)
        return false;

    FRotator CurrentCameraRotation = FollowCamera->GetComponentRotation();

    if (!bCameraMeasurementInitialized)
    {
        InitialCameraRotation = CurrentCameraRotation;
        CameraYawRotation = 0.0f;
        bCameraMeasurementInitialized = true;
        return true;
    }
    else
    {
        float DeltaYaw = CurrentCameraRotation.Yaw - InitialCameraRotation.Yaw;

        // Normalize angle to [-180, 180] range
        while (DeltaYaw > 180.0f) DeltaYaw -= 360.0f;
        while (DeltaYaw < -180.0f) DeltaYaw += 360.0f;

        CameraYawRotation = DeltaYaw;
        return true;
    }
}

// Camera-Relative Movement System with error handling
bool ASarahCharacter::UpdateCameraRotationReference()
{
    if (!FollowCamera)
    {
        LogError(TEXT("UpdateCameraRotationReference: No follow camera"));
        return false;
    }

    CurrentCameraYaw = FollowCamera->GetComponentRotation().Yaw;
    return true;
}

FVector ASarahCharacter::CalculateCameraRelativeDirection(float CameraYaw, FVector2D Input) const
{
    if (Input.IsNearlyZero())
        return FVector::ZeroVector;

    // Invert X axis for more intuitive camera-relative controls
    FVector2D InvertedInput = FVector2D(-Input.X, Input.Y);
    float InputMagnitude = InvertedInput.Size();

    // Convert input to world space direction
    float InputAngleRad = FMath::Atan2(InvertedInput.Y, InvertedInput.X);
    float CameraYawRad = FMath::DegreesToRadians(CameraYaw);

    // Adjust for UE coordinate system (forward is X, right is Y)
    float WorldAngleRad = InputAngleRad + CameraYawRad - FMath::DegreesToRadians(90.0f);

    FVector MovementDirection = FVector(
        FMath::Cos(WorldAngleRad) * InputMagnitude,
        FMath::Sin(WorldAngleRad) * InputMagnitude,
        0.0f
    );

    return MovementDirection.GetSafeNormal() * InputMagnitude;
}

bool ASarahCharacter::StartCameraRelativeMovement()
{
    // Only start camera-relative movement from idle state
    if (HighPerfFSM.current_state == STATE_IDLE && !bUsingCameraRelativeMovement)
    {
        if (!UpdateCameraRotationReference())
        {
            return false;
        }

        LockedCameraYaw = CurrentCameraYaw;
        bUsingCameraRelativeMovement = true;
        CurrentMovementAngle = LockedCameraYaw;
        TargetMovementAngle = LockedCameraYaw;
        bIsTransitioningAngle = false;
        return true;
    }
    return false;
}

void ASarahCharacter::StopCameraRelativeMovement()
{
    if (bUsingCameraRelativeMovement)
    {
        bUsingCameraRelativeMovement = false;
        bIsTransitioningAngle = false;
    }
}

// Continuous Angle Movement System
float ASarahCharacter::CalculateContinuousInputAngle() const
{
    if (!HasMovementInput()) return CurrentMovementAngle;

    // Convert input to screen-space angle
    FVector2D InvertedInput = FVector2D(-SarahMoveInput.X, SarahMoveInput.Y);
    float InputAngle = FMath::RadiansToDegrees(FMath::Atan2(InvertedInput.Y, InvertedInput.X));

    // Normalize to [0, 360) range
    if (InputAngle < 0) InputAngle += 360.0f;

    // Convert to world space angle
    float WorldAngle = InputAngle + LockedCameraYaw - 90.0f;

    // Normalize world angle
    while (WorldAngle >= 360.0f) WorldAngle -= 360.0f;
    while (WorldAngle < 0.0f) WorldAngle += 360.0f;

    return WorldAngle;
}

float ASarahCharacter::FindShortestAnglePath(float CurrentAngle, float TargetAngle) const
{
    float Difference = TargetAngle - CurrentAngle;

    // Normalize to [-180, 180] range for shortest path
    if (Difference > 180.0f) {
        Difference -= 360.0f;
    }
    else if (Difference < -180.0f) {
        Difference += 360.0f;
    }
    return Difference;
}

void ASarahCharacter::UpdateContinuousMovementAngle(float DeltaTime)
{
    if (!HasMovementInput()) return;

    // Calculate desired movement angle from input
    TargetMovementAngle = CalculateContinuousInputAngle();
    float AngleDifference = FindShortestAnglePath(CurrentMovementAngle, TargetMovementAngle);

    // Only interpolate if angle change is significant
    if (FMath::Abs(AngleDifference) > 0.4f)
    {
        bIsTransitioningAngle = true;

        // Dynamic rotation speed based on angle difference
        float TransitionSpeed = ContinuousRotationSpeed * (FMath::Abs(AngleDifference) / 180.0f) * 1.05f;
        float Progress = FMath::Clamp(FMath::Abs(AngleDifference) / 180.0f, 0.0f, 1.0f);

        // Smooth step interpolation for more natural movement
        float SmoothProgress = Progress * Progress * (3.0f - 2.0f * Progress);
        float AdjustedSpeed = TransitionSpeed * (0.7f + 0.3f * SmoothProgress);

        // Apply rotation
        float AngleStep = AngleDifference * AdjustedSpeed * DeltaTime;
        CurrentMovementAngle += AngleStep;

        // Keep angle in valid range
        while (CurrentMovementAngle >= 360.0f) CurrentMovementAngle -= 360.0f;
        while (CurrentMovementAngle < 0.0f) CurrentMovementAngle += 360.0f;

        // Snap to target when close enough
        if (FMath::Abs(FindShortestAnglePath(CurrentMovementAngle, TargetMovementAngle)) < 1.5f)
        {
            CurrentMovementAngle = TargetMovementAngle;
            bIsTransitioningAngle = false;
        }
    }
    else
    {
        // No significant change needed
        bIsTransitioningAngle = false;
        CurrentMovementAngle = TargetMovementAngle;
    }
}

bool ASarahCharacter::UpdateMovement()
{
    if (!ValidatePointers())
    {
        return false;
    }

    if (HasMovementInput())
    {
        // Start camera-relative movement system when first moving from idle
        if (HighPerfFSM.current_state == STATE_IDLE)
        {
            if (!StartCameraRelativeMovement())
            {
                return false;
            }
        }

        // Update continuous angle interpolation
        UpdateContinuousMovementAngle(GetWorld()->GetDeltaSeconds());

        FVector MovementDirection;
        if (bUsingCameraRelativeMovement)
        {
            // Use interpolated angle for smooth directional changes
            float MovementAngleRad = FMath::DegreesToRadians(CurrentMovementAngle);
            MovementDirection = FVector(
                FMath::Cos(MovementAngleRad),
                FMath::Sin(MovementAngleRad),
                0.0f
            );
        }
        else
        {
            // Standard camera-relative movement
            MovementDirection = CalculateCameraRelativeDirection(CurrentCameraYaw, SarahMoveInput);
        }

        // Apply movement with input magnitude scaling
        float InputMagnitude = SarahMoveInput.Size();
        float MovementIntensity = FMath::Clamp(InputMagnitude, 0.1f, 1.0f);
        AddMovementInput(MovementDirection, MovementIntensity);

        // Update character facing direction
        UpdateCharacterRotation();
        return true;
    }
    else
    {
        // Stop camera-relative system when no input
        StopCameraRelativeMovement();
        return true;
    }
}

FVector ASarahCharacter::GetMovementDirection() const
{
    if (!HasMovementInput())
        return FVector::ZeroVector;

    if (bUsingCameraRelativeMovement)
    {
        // Direction from interpolated angle
        float MovementAngleRad = FMath::DegreesToRadians(CurrentMovementAngle);
        return FVector(
            FMath::Cos(MovementAngleRad),
            FMath::Sin(MovementAngleRad),
            0.0f
        );
    }
    else
    {
        // Standard camera-relative direction
        return CalculateCameraRelativeDirection(CurrentCameraYaw, SarahMoveInput);
    }
}

void ASarahCharacter::UpdateCharacterRotation()
{
    FVector MovementDirection = GetMovementDirection();
    if (!MovementDirection.IsNearlyZero())
    {
        // Smoothly interpolate to face movement direction
        FRotator TargetRotation = MovementDirection.Rotation();
        FRotator CurrentRotation = GetActorRotation();
        FRotator NewRotation = FMath::RInterpTo(
            CurrentRotation,
            TargetRotation,
            GetWorld()->GetDeltaSeconds(),
            RotationInterpSpeed
        );

        // Only rotate around Z axis (yaw)
        SetActorRotation(FRotator(0, NewRotation.Yaw, 0));
    }
}

bool ASarahCharacter::HasMovementInput() const
{
    return (FMath::Abs(SarahMoveInput.X) > 0.01f || FMath::Abs(SarahMoveInput.Y) > 0.01f);
}

// Walk Transition System with error handling
bool ASarahCharacter::ShouldApplyWalkTransition() const
{
    return HighPerfFSM.previous_state == STATE_IDLE;
}

bool ASarahCharacter::StartWalkTransition()
{
    if (!WalkAnimation || !GetMesh())
    {
        LogError(TEXT("StartWalkTransition: Invalid animation or mesh"));
        return false;
    }

    if (ShouldApplyWalkTransition())
    {
        bInWalkTransition = true;
        WalkTransitionTimer = 0.0f;

        if (!C_PlayAnimationWithSpeed(WalkAnimation, WalkTransitionSpeedMultiplier))
        {
            return false;
        }
        return C_SetMovementSpeed(SarahWalkSpeed * WalkTransitionSpeedMultiplier);
    }
    else
    {
        // No transition needed for other state changes
        if (!C_SetMovementSpeed(SarahWalkSpeed))
        {
            return false;
        }
        return C_PlayAnimation(WalkAnimation);
    }
}

void ASarahCharacter::EndWalkTransition()
{
    if (bInWalkTransition)
    {
        bInWalkTransition = false;
        // Restore normal speed and animation rate
        C_PlayAnimationWithSpeed(WalkAnimation, 1.0f);
        C_SetMovementSpeed(SarahWalkSpeed);
    }
}

void ASarahCharacter::UpdateWalkTransition(float DeltaTime)
{
    if (bInWalkTransition)
    {
        WalkTransitionTimer += DeltaTime;
        float Progress = WalkTransitionTimer / WalkTransitionDuration;

        if (Progress <= 0.25f)
        {
            // Initial slow phase
            C_SetMovementSpeed(SarahWalkSpeed * WalkTransitionSpeedMultiplier);
            if (GetMesh() && CurrentAnimation == WalkAnimation)
            {
                GetMesh()->SetPlayRate(WalkTransitionSpeedMultiplier);
            }
        }
        else
        {
            // Gradual acceleration to normal speed
            float RemainingProgress = (Progress - 0.25f) / 0.75f;
            float SpeedMultiplier = FMath::Lerp(WalkTransitionSpeedMultiplier, 1.0f, RemainingProgress);

            C_SetMovementSpeed(SarahWalkSpeed * SpeedMultiplier);
            if (GetMesh() && CurrentAnimation == WalkAnimation)
            {
                GetMesh()->SetPlayRate(SpeedMultiplier);
            }
        }

        // End transition when complete
        if (Progress >= 1.0f)
        {
            EndWalkTransition();
        }
    }
}

float ASarahCharacter::GetCurrentWalkSpeed() const
{
    if (bInWalkTransition)
    {
        float Progress = WalkTransitionTimer / WalkTransitionDuration;
        if (Progress <= 0.25f)
        {
            return SarahWalkSpeed * WalkTransitionSpeedMultiplier;
        }
        else
        {
            float RemainingProgress = (Progress - 0.25f) / 0.75f;
            float SpeedMultiplier = FMath::Lerp(WalkTransitionSpeedMultiplier, 1.0f, RemainingProgress);
            return SarahWalkSpeed * SpeedMultiplier;
        }
    }
    return SarahWalkSpeed;
}

// Finite State Machine Operations with error handling
bool ASarahCharacter::FSM_ChangeState(int32_t new_state)
{
    if (new_state < 0 || new_state >= STATE_COUNT)
    {
        HighPerfFSM.error_count++;
        LogError(FString::Printf(TEXT("FSM_ChangeState: Invalid state %d"), new_state));
        return false;
    }

    int32_t current = HighPerfFSM.current_state;
    if (current == new_state) return true;

    // Execute exit logic for current state
    switch (current)
    {
    case STATE_IDLE: State_Idle_Exit(); break;
    case STATE_WALK: State_Walk_Exit(); break;
    case STATE_RUN: State_Run_Exit(); break;
    }

    // Update FSM state tracking
    HighPerfFSM.previous_state = current;
    HighPerfFSM.current_state = new_state;
    HighPerfFSM.state_transition_count[new_state]++;
    HighPerfFSM.state_enter_time[new_state] = FPlatformTime::Cycles64();

    // Execute enter logic for new state
    switch (new_state)
    {
    case STATE_IDLE: State_Idle_Enter(); break;
    case STATE_WALK: State_Walk_Enter(); break;
    case STATE_RUN: State_Run_Enter(); break;
    }

    return true;
}

bool ASarahCharacter::FSM_Update(float delta_time)
{
    if (!CheckMemorySystems())
    {
        return false;
    }

    // Profile FSM update performance
    PerformanceCounter start_cycles = FPlatformTime::Cycles64();

    bool success = true;

    // Delegate to current state's update function
    switch (HighPerfFSM.current_state)
    {
    case STATE_IDLE: success = State_Idle_Update(delta_time); break;
    case STATE_WALK: success = State_Walk_Update(delta_time); break;
    case STATE_RUN: success = State_Run_Update(delta_time); break;
    default:
        success = false;
        LogError(TEXT("FSM_Update: Invalid current state"));
        break;
    }

    PerformanceCounter end_cycles = FPlatformTime::Cycles64();
    HighPerfFSM.last_update_cycles = end_cycles - start_cycles;
    HighPerfFSM.total_update_cycles += HighPerfFSM.last_update_cycles;
    HighPerfFSM.update_count++;

    PerformanceMetrics.fsm_update_cycles += HighPerfFSM.last_update_cycles;
    return success;
}

// State Implementation - Idle
void ASarahCharacter::State_Idle_Enter()
{
    C_SetMovementSpeed(0.0f);
    C_PlayAnimation(IdleAnimation);
    bInWalkTransition = false;
}

bool ASarahCharacter::State_Idle_Update(float delta_time)
{
    bool hasMovementInput = HasMovementInput();
    if (hasMovementInput)
    {
        if (bSarahIsSprinting)
        {
            return FSM_ChangeState(STATE_RUN);
        }
        else
        {
            return FSM_ChangeState(STATE_WALK);
        }
    }
    return true;
}

void ASarahCharacter::State_Idle_Exit()
{
    // No special cleanup needed for idle exit
}

// State Implementation - Walk
void ASarahCharacter::State_Walk_Enter()
{
    StartWalkTransition();
}

bool ASarahCharacter::State_Walk_Update(float delta_time)
{
    bool hasMovementInput = HasMovementInput();
    UpdateWalkTransition(delta_time);

    if (!hasMovementInput)
    {
        return FSM_ChangeState(STATE_IDLE);
    }
    else if (bSarahIsSprinting)
    {
        return FSM_ChangeState(STATE_RUN);
    }
    return true;
}

void ASarahCharacter::State_Walk_Exit()
{
    if (bInWalkTransition)
    {
        EndWalkTransition();
    }
}

// State Implementation - Run
void ASarahCharacter::State_Run_Enter()
{
    if (!C_SetMovementSpeed(SarahRunSpeed))
    {
        LogError(TEXT("State_Run_Enter: Failed to set movement speed"));
    }

    if (!C_PlayAnimation(RunAnimation))
    {
        LogError(TEXT("State_Run_Enter: Failed to play run animation"));
    }

    // Ensure any walk transition is cleaned up
    if (bInWalkTransition)
    {
        EndWalkTransition();
    }
}

bool ASarahCharacter::State_Run_Update(float delta_time)
{
    bool hasMovementInput = HasMovementInput();
    if (!hasMovementInput)
    {
        return FSM_ChangeState(STATE_IDLE);
    }
    else if (!bSarahIsSprinting)
    {
        return FSM_ChangeState(STATE_WALK);
    }
    return true;
}

void ASarahCharacter::State_Run_Exit()
{
    // No special cleanup needed for run exit
}

// Low-Level Control Functions with error handling
void ASarahCharacter::C_StopMovement()
{
    if (GetCharacterMovement())
    {
        GetCharacterMovement()->StopMovementImmediately();
    }
}

bool ASarahCharacter::C_SetMovementSpeed(float Speed)
{
    if (!GetCharacterMovement())
    {
        LogError(TEXT("C_SetMovementSpeed: No character movement component"));
        return false;
    }

    GetCharacterMovement()->MaxWalkSpeed = Speed;
    return true;
}

bool ASarahCharacter::C_PlayAnimation(UAnimSequence* Animation)
{
    if (!Animation || !GetMesh())
    {
        LogError(TEXT("C_PlayAnimation: Invalid animation or mesh"));
        return false;
    }

    // Stop current animation before playing new one
    if (CurrentAnimation) GetMesh()->Stop();

    GetMesh()->PlayAnimation(Animation, true);
    CurrentAnimation = Animation;
    GetMesh()->SetPlayRate(1.0f);
    return true;
}

bool ASarahCharacter::C_PlayAnimationWithSpeed(UAnimSequence* Animation, float Speed)
{
    if (!Animation || !GetMesh())
    {
        LogError(TEXT("C_PlayAnimationWithSpeed: Invalid animation or mesh"));
        return false;
    }

    if (CurrentAnimation) GetMesh()->Stop();

    GetMesh()->PlayAnimation(Animation, true);
    CurrentAnimation = Animation;
    GetMesh()->SetPlayRate(Speed);
    return true;
}

bool ASarahCharacter::UpdateCStateData()
{
    if (!ValidatePointers())
    {
        return false;
    }

    // Profile state data update for performance monitoring
    PerformanceCounter start_cycles = FPlatformTime::Cycles64();

    PreviousStateData = CurrentStateData;
    CurrentStateData.move_x = SarahMoveInput.X;
    CurrentStateData.move_y = SarahMoveInput.Y;
    CurrentStateData.is_sprinting = bSarahIsSprinting;

    // Capture current velocity
    FVector Velocity = GetVelocity();
    CurrentStateData.velocity_x = Velocity.X;
    CurrentStateData.velocity_y = Velocity.Y;
    CurrentStateData.velocity_z = Velocity.Z;

    // Track movement state changes
    CurrentStateData.was_moving = PreviousStateData.move_x != 0.0f || PreviousStateData.move_y != 0.0f;

    PerformanceCounter end_cycles = FPlatformTime::Cycles64();
    PerformanceMetrics.state_data_update_cycles = end_cycles - start_cycles;
    return true;
}

// Performance Monitoring
void ASarahCharacter::UpdatePerformanceMetrics(float delta_time)
{
    if (!bPerformanceMetricsEnabled) return;

    // Calculate average FSM update time
    if (HighPerfFSM.update_count > 0)
    {
        double avg_update_cycles = (double)HighPerfFSM.total_update_cycles / HighPerfFSM.update_count;
        PerformanceMetrics.avg_fsm_update_time_ms = FPlatformTime::ToMilliseconds(avg_update_cycles);
    }

    PerformanceMetrics.avg_frame_time_ms = delta_time * 1000.0f;
}

void ASarahCharacter::PrintPerformanceMetrics()
{
    UE_LOG(LogTemp, Warning, TEXT("Performance Metrics:"));
    UE_LOG(LogTemp, Warning, TEXT("  FSM Update Time: %.3f ms"), PerformanceMetrics.avg_fsm_update_time_ms);
    UE_LOG(LogTemp, Warning, TEXT("  Frame Time: %.3f ms"), PerformanceMetrics.avg_frame_time_ms);
    UE_LOG(LogTemp, Warning, TEXT("  Memory Pool Hits: %d"), PerformanceMetrics.memory_pool_hits);
    UE_LOG(LogTemp, Warning, TEXT("  Memory Pool Misses: %d"), PerformanceMetrics.memory_pool_misses);
    UE_LOG(LogTemp, Warning, TEXT("  Stack Arena Overflows: %d"), PerformanceMetrics.stack_arena_overflows);
    UE_LOG(LogTemp, Warning, TEXT("  FSM Errors: %d"), HighPerfFSM.error_count);
}

// Error handling utilities
bool ASarahCharacter::ValidatePointers() const
{
    bool valid = true;

    if (!GetMesh())
    {
        LogError(TEXT("ValidatePointers: Mesh is null"));
        valid = false;
    }

    if (!GetCharacterMovement())
    {
        LogError(TEXT("ValidatePointers: CharacterMovement is null"));
        valid = false;
    }

    if (!FollowCamera)
    {
        LogError(TEXT("ValidatePointers: FollowCamera is null"));
        valid = false;
    }

    if (!CameraBoom)
    {
        LogError(TEXT("ValidatePointers: CameraBoom is null"));
        valid = false;
    }

    return valid;
}

void ASarahCharacter::LogError(const FString& ErrorMessage) const
{
    UE_LOG(LogTemp, Error, TEXT("SarahCharacter Error: %s"), *ErrorMessage);
}

bool ASarahCharacter::CheckMemorySystems() const
{
    bool valid = true;

    if (!StateMemoryPool)
    {
        LogError(TEXT("CheckMemorySystems: StateMemoryPool is null"));
        valid = false;
    }

    if (!FrameArena)
    {
        LogError(TEXT("CheckMemorySystems: FrameArena is null"));
        valid = false;
    }

    return valid;
}

// Public Animation Interface with error handling
bool ASarahCharacter::PlayAnimationDirect(UAnimSequence* Animation)
{
    return C_PlayAnimation(Animation);
}

void ASarahCharacter::StopAnimationDirect()
{
    if (GetMesh())
    {
        GetMesh()->Stop();
        CurrentAnimation = nullptr;
    }
}

// Constructor and Lifecycle
ASarahCharacter::ASarahCharacter()
{
    PrimaryActorTick.bCanEverTick = true;

    // Initialize performance systems with error handling
    if (!CreateMemoryPool(&StateMemoryPool, 128, 8))
    {
        LogError(TEXT("Constructor: Failed to create memory pool"));
    }

    if (!CreateStackArena(&FrameArena, 1024 * 16))
    {
        LogError(TEXT("Constructor: Failed to create stack arena"));
    }

    // Zero-initialize all data structures
    FMemory::Memzero(&CurrentStateData, sizeof(FStateData));
    FMemory::Memzero(&PreviousStateData, sizeof(FStateData));
    FMemory::Memzero(&HighPerfFSM, sizeof(FHighPerfFSM));
    FMemory::Memzero(&PerformanceMetrics, sizeof(FPerformanceMetrics));

    CurrentFrameID = 0;
    bPerformanceMetricsEnabled = true;

    // Initialize movement system state
    CurrentCameraYaw = 0.0f;
    LockedCameraYaw = 0.0f;
    bUsingCameraRelativeMovement = false;
    CurrentMovementAngle = 0.0f;
    TargetMovementAngle = 0.0f;
    bIsTransitioningAngle = false;

    // Initialize camera measurement system
    bCameraMeasurementActive = false;
    CameraYawRotation = 0.0f;
    bCameraMeasurementInitialized = false;
    InitialCameraRotation = FRotator::ZeroRotator;

    // Initialize animation transition system
    bInWalkTransition = false;
    WalkTransitionTimer = 0.0f;

    // Setup character capsule collision
    GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);

    // Load character mesh with error handling
    static ConstructorHelpers::FObjectFinder<USkeletalMesh> SkeletalMeshFinder(TEXT("/Game/Adventure_Pack/Characters/Sarah/Mesh/SK_Sarah"));
    if (SkeletalMeshFinder.Succeeded())
    {
        GetMesh()->SetSkeletalMesh(SkeletalMeshFinder.Object);
        GetMesh()->SetAnimInstanceClass(nullptr);
    }
    else
    {
        LogError(TEXT("Constructor: Failed to load skeletal mesh"));
    }

    // Position mesh relative to capsule
    GetMesh()->SetRelativeLocation(FVector(0.0f, 0.0f, -90.0f));
    GetMesh()->SetRelativeRotation(FRotator(0.0f, -90.0f, 0.0f));

    // Create camera boom (spring arm)
    CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
    if (CameraBoom)
    {
        CameraBoom->SetupAttachment(RootComponent);
        CameraBoom->TargetArmLength = 400.0f;
        CameraBoom->bUsePawnControlRotation = true;
        CameraBoom->bInheritPitch = true;
        CameraBoom->bInheritYaw = true;
        CameraBoom->bInheritRoll = true;
    }

    // Create follow camera
    FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
    if (FollowCamera)
    {
        FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
        FollowCamera->bUsePawnControlRotation = false;
    }

    // Configure character movement
    bUseControllerRotationPitch = false;
    bUseControllerRotationYaw = false;
    bUseControllerRotationRoll = false;

    if (GetCharacterMovement())
    {
        GetCharacterMovement()->bOrientRotationToMovement = true;
        GetCharacterMovement()->RotationRate = FRotator(0.0f, 540.0f, 0.0f);
        GetCharacterMovement()->JumpZVelocity = 600.f;
        GetCharacterMovement()->AirControl = 0.2f;
        GetCharacterMovement()->MaxWalkSpeed = SarahWalkSpeed;
    }

    // Load input assets with error handling
    static ConstructorHelpers::FObjectFinder<UInputMappingContext> IMC_Finder(TEXT("/Game/Sarah/Inputs/IMC_Sarah"));
    if (IMC_Finder.Succeeded())
    {
        SarahDefaultMappingContext = IMC_Finder.Object;
    }

    static ConstructorHelpers::FObjectFinder<UInputAction> MoveAction_Finder(TEXT("/Game/Sarah/Inputs/IA_Sarah_Move"));
    if (MoveAction_Finder.Succeeded())
    {
        SarahMoveAction = MoveAction_Finder.Object;
    }

    static ConstructorHelpers::FObjectFinder<UInputAction> LookAction_Finder(TEXT("/Game/Sarah/Inputs/IA_Sarah_Look"));
    if (LookAction_Finder.Succeeded())
    {
        SarahLookAction = LookAction_Finder.Object;
    }

    static ConstructorHelpers::FObjectFinder<UInputAction> SprintAction_Finder(TEXT("/Game/Sarah/Inputs/IA_Sarah_Sprint"));
    if (SprintAction_Finder.Succeeded())
    {
        SarahSprintAction = SprintAction_Finder.Object;
    }

    // Load animation sequences with error handling
    static ConstructorHelpers::FObjectFinder<UAnimSequence> IdleAnimFinder(TEXT("/Game/Adventure_Pack/Characters/Sarah/Animation/AS_Sarah_MF_Idle"));
    if (IdleAnimFinder.Succeeded())
    {
        IdleAnimation = IdleAnimFinder.Object;
    }

    static ConstructorHelpers::FObjectFinder<UAnimSequence> WalkAnimFinder(TEXT("/Game/Adventure_Pack/Characters/Sarah/Animation/AS_Sarah_MF_Walk_Fwd"));
    if (WalkAnimFinder.Succeeded())
    {
        WalkAnimation = WalkAnimFinder.Object;
    }

    static ConstructorHelpers::FObjectFinder<UAnimSequence> RunAnimFinder(TEXT("/Game/Adventure_Pack/Characters/Sarah/Animation/AS_Sarah_MF_Run_Fwd"));
    if (RunAnimFinder.Succeeded())
    {
        RunAnimation = RunAnimFinder.Object;
    }
}

ASarahCharacter::~ASarahCharacter()
{
    // Clean up custom memory systems
    DestroyMemoryPool(StateMemoryPool);
    DestroyStackArena(FrameArena);
}

void ASarahCharacter::BeginPlay()
{
    Super::BeginPlay();

    // Setup Enhanced Input system with error handling
    if (APlayerController* PlayerController = Cast<APlayerController>(Controller))
    {
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PlayerController->GetLocalPlayer()))
        {
            if (SarahDefaultMappingContext)
            {
                Subsystem->AddMappingContext(SarahDefaultMappingContext, 0);
            }
            else
            {
                LogError(TEXT("BeginPlay: No default mapping context"));
            }
        }
    }

    // Initialize movement systems with current camera state
    if (FollowCamera)
    {
        CurrentCameraYaw = FollowCamera->GetComponentRotation().Yaw;
        CurrentMovementAngle = CurrentCameraYaw;
        TargetMovementAngle = CurrentCameraYaw;
    }

    // Start systems
    StartCameraRotationMeasurement();
    FSM_ChangeState(STATE_IDLE);
}

void ASarahCharacter::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Update frame tracking for memory management
    CurrentFrameID++;
    ArenaResetFrame(FrameArena, CurrentFrameID);

    // Profile frame timing
    PerformanceCounter frame_start_cycles = FPlatformTime::Cycles64();

    // Update all systems in order of dependency with error handling
    if (!UpdateCameraRotationReference())
    {
        LogError(TEXT("Tick: Failed to update camera rotation reference"));
    }

    if (!UpdateCStateData())
    {
        LogError(TEXT("Tick: Failed to update state data"));
    }

    if (!UpdateCameraRotationMeasurement())
    {
        // This is not always an error - measurement might not be active
    }

    if (!UpdateMovement())
    {
        LogError(TEXT("Tick: Failed to update movement"));
    }

    if (!FSM_Update(DeltaTime))
    {
        LogError(TEXT("Tick: FSM update failed"));
    }

    // Update performance metrics
    PerformanceMetrics.total_frame_cycles = FPlatformTime::Cycles64() - frame_start_cycles;
    UpdatePerformanceMetrics(DeltaTime);
}

// Input System Setup
void ASarahCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    if (UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(PlayerInputComponent))
    {
        // Bind movement actions with null checks
        if (SarahMoveAction)
        {
            EnhancedInputComponent->BindAction(SarahMoveAction, ETriggerEvent::Triggered, this, &ASarahCharacter::SarahMove);
            EnhancedInputComponent->BindAction(SarahMoveAction, ETriggerEvent::Completed, this, &ASarahCharacter::SarahMoveStop);
        }

        // Bind look action
        if (SarahLookAction)
        {
            EnhancedInputComponent->BindAction(SarahLookAction, ETriggerEvent::Triggered, this, &ASarahCharacter::SarahLook);
        }

        // Bind sprint actions
        if (SarahSprintAction)
        {
            EnhancedInputComponent->BindAction(SarahSprintAction, ETriggerEvent::Started, this, &ASarahCharacter::SarahStartSprint);
            EnhancedInputComponent->BindAction(SarahSprintAction, ETriggerEvent::Completed, this, &ASarahCharacter::SarahStopSprint);
        }
    }
}

// Input Handlers
void ASarahCharacter::SarahMove(const FInputActionValue& Value)
{
    FVector2D PreviousInput = SarahMoveInput;
    FVector2D RawInput = Value.Get<FVector2D>();
    SarahMoveInput = RawInput;

    // Normalize input if magnitude exceeds 1.0 (gamepad circles)
    float InputMagnitude = SarahMoveInput.Size();
    if (InputMagnitude > 1.0f)
    {
        SarahMoveInput = SarahMoveInput.GetSafeNormal();
    }
}

void ASarahCharacter::SarahMoveStop(const FInputActionValue& Value)
{
    SarahMoveInput = FVector2D::ZeroVector;
    bIsTransitioningAngle = false;
}

void ASarahCharacter::SarahLook(const FInputActionValue& Value)
{
    FVector2D LookAxisVector = Value.Get<FVector2D>();

    if (Controller != nullptr)
    {
        // Apply look input with frame rate independent scaling
        AddControllerYawInput(LookAxisVector.X * BaseTurnRate * GetWorld()->GetDeltaSeconds());
        AddControllerPitchInput(LookAxisVector.Y * BaseLookUpRate * GetWorld()->GetDeltaSeconds());

        // Clamp pitch to prevent over-rotation
        FRotator ControlRotation = GetControlRotation();
        ControlRotation.Pitch = FMath::Clamp(ControlRotation.Pitch, -89.0f, 89.0f);
        Controller->SetControlRotation(ControlRotation);
    }
}

void ASarahCharacter::SarahStartSprint()
{
    bSarahIsSprinting = true;
}

void ASarahCharacter::SarahStopSprint()
{
    bSarahIsSprinting = false;
}

FString ASarahCharacter::GetMovementDirectionName() const
{
    if (!HasMovementInput()) return TEXT("None");
    return FString::Printf(TEXT("%.1f°"), CurrentMovementAngle);
}

FString ASarahCharacter::GetWASDButtonsPressed() const
{
    FString Buttons;
    if (SarahMoveInput.Y > 0.0f) Buttons += "W";
    if (SarahMoveInput.Y < 0.0f) Buttons += "S";
    if (SarahMoveInput.X > 0.0f) Buttons += "D";
    if (SarahMoveInput.X < 0.0f) Buttons += "A";
    return Buttons.IsEmpty() ? TEXT("None") : Buttons;
}
