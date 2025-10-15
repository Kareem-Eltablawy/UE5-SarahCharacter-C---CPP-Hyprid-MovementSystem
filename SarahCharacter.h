#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "InputActionValue.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "EnhancedInput/Public/InputMappingContext.h"
#include "EnhancedInput/Public/InputAction.h"
#include "Animation/AnimSequence.h"
#include "SarahCharacter.generated.h"

// Performance counter types
typedef uint64 PerformanceCounter;
typedef uint32 FrameID;

/**
 * Character movement states for the finite state machine
 */
typedef enum EStateId {
    STATE_IDLE = 0,
    STATE_WALK,
    STATE_RUN,
    STATE_COUNT
} EStateId;

/**
 * Simple memory pool for frequent allocations in performance-critical paths
 */
typedef struct FMemoryPool {
    void* memory;
    void** free_list;
    size_t block_size;
    size_t block_count;
    size_t free_count;
    uint32_t allocation_count;
    uint32_t free_count_total;
} FMemoryPool;

/**
 * Frame-based stack allocator for temporary data
 */
typedef struct FStackArena {
    uint8_t* memory;
    size_t size;
    size_t offset;
    FrameID frame_id;
    size_t peak_usage;
} FStackArena;

/**
 * Cache-aligned state data structure to prevent false sharing
 */
typedef struct alignas(64) FStateData {
    float move_x;
    float move_y;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    bool is_sprinting;
    bool is_moving;
    bool was_moving;
    uint8_t padding[37]; // Pad to 64 bytes for cache line alignment
} FStateData;

/**
 * High-performance finite state machine with profiling
 */
typedef struct FHighPerfFSM {
    int32_t current_state;
    int32_t previous_state;
    PerformanceCounter state_enter_time[STATE_COUNT];
    uint32_t state_transition_count[STATE_COUNT];
    PerformanceCounter last_update_cycles;
    PerformanceCounter total_update_cycles;
    uint32_t update_count;
    uint32_t error_count;
} FHighPerfFSM;

/**
 * Performance tracking for optimization
 */
typedef struct FPerformanceMetrics {
    PerformanceCounter fsm_update_cycles;
    PerformanceCounter state_data_update_cycles;
    PerformanceCounter total_frame_cycles;
    PerformanceCounter memory_allocation_cycles;
    float avg_fsm_update_time_ms;
    float avg_frame_time_ms;
    uint32_t memory_pool_hits;
    uint32_t memory_pool_misses;
    uint32_t stack_arena_overflows;
} FPerformanceMetrics;

UCLASS()
class SARAH_API ASarahCharacter : public ACharacter
{
    GENERATED_BODY()

public:
    ASarahCharacter();
    virtual ~ASarahCharacter();

    // Camera Components
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    USpringArmComponent* CameraBoom;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    UCameraComponent* FollowCamera;

    // Animation control functions
    UFUNCTION(BlueprintCallable, Category = "Sarah|Animation")
    bool PlayAnimationDirect(UAnimSequence* Animation);

    UFUNCTION(BlueprintCallable, Category = "Sarah|Animation")
    void StopAnimationDirect();

    // State query functions
    UFUNCTION(BlueprintPure, Category = "SarahFSM")
    bool SarahIsIdle() const { return HighPerfFSM.current_state == STATE_IDLE; }

    UFUNCTION(BlueprintPure, Category = "SarahFSM")
    bool SarahIsWalking() const { return HighPerfFSM.current_state == STATE_WALK; }

    UFUNCTION(BlueprintPure, Category = "SarahFSM")
    bool SarahIsRunning() const { return HighPerfFSM.current_state == STATE_RUN; }

    UFUNCTION(BlueprintCallable, Category = "Sarah|Movement")
    FString GetMovementDirectionName() const;

    UFUNCTION(BlueprintCallable, Category = "Sarah|Performance")
    void PrintPerformanceMetrics();

    // Camera rotation measurement
    UFUNCTION(BlueprintCallable, Category = "Sarah|Camera")
    bool StartCameraRotationMeasurement();

    UFUNCTION(BlueprintCallable, Category = "Sarah|Camera")
    void StopCameraRotationMeasurement();

    UFUNCTION(BlueprintPure, Category = "Sarah|Camera")
    float GetCameraYawRotation() const { return CameraYawRotation; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Camera")
    bool IsCameraMeasurementActive() const { return bCameraMeasurementActive; }

    // Movement state getters
    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    FVector2D GetSarahMoveInput() const { return SarahMoveInput; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    bool GetSarahIsSprinting() const { return bSarahIsSprinting; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    float GetSarahWalkSpeed() const { return SarahWalkSpeed; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    float GetSarahRunSpeed() const { return SarahRunSpeed; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    float GetCurrentCameraYaw() const { return CurrentCameraYaw; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    float GetLockedCameraYaw() const { return LockedCameraYaw; }

    UFUNCTION(BlueprintPure, Category = "Sarah|Movement")
    bool IsUsingCameraRelativeMovement() const { return bUsingCameraRelativeMovement; }

    // Configurable movement properties
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sarah|Movement")
    float ContinuousRotationSpeed = 8.0f;

    // Animation assets
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sarah|Animation")
    UAnimSequence* IdleAnimation;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sarah|Animation")
    UAnimSequence* WalkAnimation;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sarah|Animation")
    UAnimSequence* RunAnimation;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    // Enhanced Input System assets
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputMappingContext* SarahDefaultMappingContext;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* SarahMoveAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* SarahLookAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
    UInputAction* SarahSprintAction;

    // Movement configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float SarahWalkSpeed = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float SarahRunSpeed = 600.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
    FVector2D SarahMoveInput;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
    bool bSarahIsSprinting = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float RotationInterpSpeed = 13.0f;

    // Camera control settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    float BaseTurnRate = 45.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    float BaseLookUpRate = 45.0f;

private:
    // Performance optimization systems
    FMemoryPool* StateMemoryPool;
    FStackArena* FrameArena;
    FHighPerfFSM HighPerfFSM;
    FStateData CurrentStateData;
    FStateData PreviousStateData;
    FPerformanceMetrics PerformanceMetrics;
    FrameID CurrentFrameID;
    bool bPerformanceMetricsEnabled;

    // Animation system
    UPROPERTY()
    UAnimSequence* CurrentAnimation;

    // Camera-relative movement system
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement", meta = (AllowPrivateAccess = "true"))
    float CurrentCameraYaw;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement", meta = (AllowPrivateAccess = "true"))
    float LockedCameraYaw;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement", meta = (AllowPrivateAccess = "true"))
    bool bUsingCameraRelativeMovement;

    // Continuous angle movement
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement", meta = (AllowPrivateAccess = "true"))
    float CurrentMovementAngle;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement", meta = (AllowPrivateAccess = "true"))
    float TargetMovementAngle;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement", meta = (AllowPrivateAccess = "true"))
    bool bIsTransitioningAngle;

    // Camera measurement
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera", meta = (AllowPrivateAccess = "true"))
    bool bCameraMeasurementActive;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera", meta = (AllowPrivateAccess = "true"))
    float CameraYawRotation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera", meta = (AllowPrivateAccess = "true"))
    FRotator InitialCameraRotation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera", meta = (AllowPrivateAccess = "true"))
    bool bCameraMeasurementInitialized;

    // Animation transition system
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Animation", meta = (AllowPrivateAccess = "true"))
    bool bInWalkTransition;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Animation", meta = (AllowPrivateAccess = "true"))
    float WalkTransitionTimer;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Animation", meta = (AllowPrivateAccess = "true"))
    float WalkTransitionDuration = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Animation", meta = (AllowPrivateAccess = "true"))
    float WalkTransitionSpeedMultiplier = 0.85f;

    // Input callback handlers - C-style naming
    void SarahMove(const FInputActionValue& Value);
    void SarahMoveStop(const FInputActionValue& Value);
    void SarahLook(const FInputActionValue& Value);
    void SarahStartSprint();
    void SarahStopSprint();

    // Memory management - C-style implementation
    bool CreateMemoryPool(FMemoryPool** OutPool, size_t block_size, size_t block_count);
    void DestroyMemoryPool(FMemoryPool* pool);
    void* PoolAlloc(FMemoryPool* pool);
    bool PoolFree(FMemoryPool* pool, void* block);
    bool CreateStackArena(FStackArena** OutArena, size_t size);
    void DestroyStackArena(FStackArena* arena);
    void* ArenaAlloc(FStackArena* arena, size_t size);
    void ArenaResetFrame(FStackArena* arena, FrameID frame_id);

    // Finite State Machine operations
    bool FSM_ChangeState(int32_t new_state);
    bool FSM_Update(float delta_time);

    // State lifecycle functions
    void State_Idle_Enter();
    bool State_Idle_Update(float delta_time);
    void State_Idle_Exit();
    void State_Walk_Enter();
    bool State_Walk_Update(float delta_time);
    void State_Walk_Exit();
    void State_Run_Enter();
    bool State_Run_Update(float delta_time);
    void State_Run_Exit();

    // Low-level movement and animation control
    void C_StopMovement();
    bool C_SetMovementSpeed(float Speed);
    bool C_PlayAnimation(UAnimSequence* Animation);
    bool C_PlayAnimationWithSpeed(UAnimSequence* Animation, float Speed);

    // Camera-relative movement implementation
    bool UpdateCameraRotationReference();
    FVector CalculateCameraRelativeDirection(float CameraYaw, FVector2D Input) const;
    bool StartCameraRelativeMovement();
    void StopCameraRelativeMovement();
    bool UpdateMovement();
    FVector GetMovementDirection() const;
    void UpdateCharacterRotation();
    bool HasMovementInput() const;

    // Continuous angle calculation
    float CalculateContinuousInputAngle() const;
    void UpdateContinuousMovementAngle(float DeltaTime);
    float FindShortestAnglePath(float CurrentAngle, float TargetAngle) const;

    // Camera measurement utilities
    bool UpdateCameraRotationMeasurement();
    void DebugCameraRotation();

    // Animation transition management
    void UpdateWalkTransition(float DeltaTime);
    bool StartWalkTransition();
    void EndWalkTransition();
    float GetCurrentWalkSpeed() const;
    bool ShouldApplyWalkTransition() const;

    // Performance profiling and metrics
    void UpdatePerformanceMetrics(float delta_time);
    bool UpdateCStateData();

    // Debug utilities
    FString GetWASDButtonsPressed() const;

    // Error handling
    bool ValidatePointers() const;
    void LogError(const FString& ErrorMessage) const;
    bool CheckMemorySystems() const;
};
