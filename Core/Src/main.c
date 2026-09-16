/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Left-wall-following micromouse (state-machine navigation)
  ******************************************************************************
  *
  * NAVIGATION DESIGN (per spec):
  *   Priority each control cycle, highest first:
  *     1. EXIT DETECTION   : FRONT_VERY_OPEN && LEFT_WALL_PRESENT,
  *                            confirmed over EXIT_CONFIRMATION_COUNT samples
 *                            -> stop permanently.
  *     2. LEFT PATH        : LEFT_OPEN, confirmed over
  *                            LEFT_OPEN_CONFIRM_COUNT samples -> creep
  *                            forward to the intersection, turn left.
  *     3. FRONT PATH       : FRONT_OPEN -> wall-follow at
  *                            LEFT_TARGET_DISTANCE_MM +/- LEFT_TOLERANCE_MM
  *                            using differential motor speed correction.
  *     4. BLOCKED          : FRONT_BLOCKED && LEFT_WALL_PRESENT -> check
  *                            RIGHT: turn right if open, else U-turn.
  *
  * CALIBRATION FLAGS -- READ BEFORE FIRST RUN:
 *   - Sensor offsets and SIDE_TARGET_DISTANCE_MM are intentionally starting
 *     values. Derive every offset from a ruler measurement, then tune the
 *     side target in a real corridor.
  *   - TURN_90_TIME_MS / TURN_180_TIME_MS are unmeasured placeholders and
  *     MUST be experimentally tuned on the real robot (spec section 11).
  *   - Per spec section 17: verify Motor_Forward()/Backward()/TurnLeft()/
  *     TurnRight()/UTurn() all produce the CORRECT physical motion before
  *     enabling autonomous navigation. If one side is wrong, flip that
  *     side's LEFT_MOTOR_INVERTED / RIGHT_MOTOR_INVERTED constant below --
  *     no other code needs to change.
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "VL53L0X.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/**
 * @brief Navigation state, tracked for diagnostics/clarity per spec section 15.
 *        Most transitions execute a full calibrated blocking maneuver before
 *        returning to NAV_FOLLOW_WALL -- there is no separate scheduler, so
 *        this enum documents intent rather than driving a fully non-blocking
 *        dispatcher.
 */
typedef enum {
    NAV_FOLLOW_WALL = 0,
    NAV_APPROACH_LEFT_TURN,
    NAV_TURN_LEFT,
    NAV_APPROACH_RIGHT_TURN,
    NAV_TURN_RIGHT,
    NAV_U_TURN,
    NAV_EXIT_DETECTED,
    NAV_STOPPED
} NavState_t;

typedef enum {
    SENSOR_OK = 0,
    SENSOR_FAR,
    SENSOR_TIMEOUT,
    SENSOR_RANGE_ERROR,
    SENSOR_I2C_ERROR
} SensorStatus_t;

typedef struct {
    uint16_t mm;              /* corrected; FAR is represented as MAX_VALID */
    SensorStatus_t status;
} SensorSample_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* ============================================================
 * TCA9548A
 * ============================================================ */

#define TCA9548A_ADDR       (0x70 << 1)

/*
 * Sensor mapping:
 *
 * CH0 -> LEFT   -> SD0 / SC0
 * CH1 -> FRONT  -> SD1 / SC1
 * CH2 -> RIGHT  -> SD2 / SC2
 */

#define TCA_LEFT_CHANNEL    0x01
#define TCA_FRONT_CHANNEL   0x02
#define TCA_RIGHT_CHANNEL   0x04

#define TCA_I2C_TIMEOUT_MS  20   // Finite timeout instead of HAL_MAX_DELAY --
                                  // avoids an indefinite hang if a wire comes
                                  // loose mid-run.


/* ============================================================
 * VL53L0X
 * ============================================================ */

statInfo_t_VL53L0X leftStats;
statInfo_t_VL53L0X frontStats;
statInfo_t_VL53L0X rightStats;

uint16_t leftDistance;
uint16_t frontDistance;
uint16_t rightDistance;
static SensorSample_t leftSample, frontSample, rightSample;

/* Per-sensor calibration offsets (mm). See CALIBRATION FLAGS above. */
#define LEFT_SENSOR_OFFSET_MM    0  /* actual_mm - measured_mm; calibrate */
#define FRONT_SENSOR_OFFSET_MM   0  /* independently; do not copy left offset */
#define RIGHT_SENSOR_OFFSET_MM   0  /* independently; do not copy left offset */

/* Sentinel raw value the VL53L0X port returns on timeout/out-of-range.
 * Verify this matches your exact port if exit detection misbehaves --
 * some ports return 0, others 65535, others a large fixed sentinel. */
#define SENSOR_TIMEOUT_SENTINEL_MM 8000


/* ============================================================
 * Sensor validity range and filtering
 * ============================================================ */

#define MIN_VALID_DISTANCE_MM     90
#define MAX_VALID_DISTANCE_MM     600

#define MEDIAN_FILTER_SAMPLES     3   // 3-sample median filter per sensor


/* ============================================================
 * Motor PWM
 * ============================================================ */

/*
 * TIM5_CH1 / PA0 -> LEFT motor PWMA
 * TIM2_CH1 / PA5 -> RIGHT motor PWMB
 *
 * Timer Period = 49
 *
 * 0  = 0% PWM
 * 49 = 100% PWM
 */

#define PWM_MAX                     49U
#define BASE_SPEED                  18U     /* conservative first-run speed */
#define TURN_SPEED                  20U
#define MIN_SPEED                   8U
#define MAX_SPEED                   PWM_MAX
#define WALL_KP_NUMERATOR           1       /* PWM counts per 12 mm */
#define WALL_KP_DENOMINATOR         12
#define WALL_DEADBAND_MM            8
#define MAX_WALL_CORRECTION         7
/* Set to -1 if a positive L-R error steers the wrong physical direction. */
#define WALL_STEERING_SIGN          1

/* Bench-test drive: Motor_Forward(BASE_SPEED, BASE_SPEED) should move the
 * robot straight forward, Motor_TurnLeft() should rotate it counter-
 * clockwise, Motor_TurnRight() clockwise. If one side disagrees, flip that
 * side's constant below -- no other code needs to change (spec section 17). */
#define LEFT_MOTOR_INVERTED    0
#define RIGHT_MOTOR_INVERTED   0


/* ============================================================
 * Wall-following thresholds
 * ============================================================ */

/*
 * Conservative starting values only. Tune them in the physical maze.
 */

/* Front wall considered blocked at or below this distance */
#define FRONT_BLOCKED_DISTANCE_MM       115
#define FRONT_OPEN_DISTANCE_MM          140

/* Left side considered open (possible path) above this distance */
#define LEFT_WALL_PRESENT_THRESHOLD_MM  170
#define LEFT_OPEN_DISTANCE_MM           240
#define RIGHT_OPEN_DISTANCE_MM          240

/* Desired left-wall distance and tolerance band */
#define SIDE_TARGET_DISTANCE_MM         115 /* tune in the real maze */


/* ============================================================
 * Confirmation counts (reject transient/noisy readings)
 * ============================================================ */

#define LEFT_OPEN_CONFIRM_COUNT   3   // Consecutive LEFT_OPEN reads before committing to a left turn
#define RIGHT_OPEN_CONFIRM_COUNT  2
#define FRONT_BLOCKED_CONFIRM_COUNT 2
#define EXIT_CONFIRMATION_COUNT   4   // Consecutive exit-condition reads before stopping permanently
#define SENSOR_FAULT_CONFIRM_COUNT 3

/* DEBUG_MODE requires a project-specific printf/UART/SWO retarget.  It is
 * deliberately off by default because this CubeMX project has no UART. */
#define DEBUG_MODE 0


/* ============================================================
 * Turn / maneuver timing (spec section 11 -- MUST be tuned)
 * ============================================================ */

#define APPROACH_CREEP_MS    60    // Short forward creep toward intersection center before turning
#define STOP_PAUSE_MS         40    // Brief stop before executing any turn
#define TURN_90_TIME_MS      350
#define TURN_180_TIME_MS     700


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */

uint8_t TCA_SelectChannel(uint8_t channel);

uint8_t Sensors_Init(void);

SensorSample_t ReadLeft(void);
SensorSample_t ReadFront(void);
SensorSample_t ReadRight(void);

void ReadAllSensors(void);

void Motors_Init(void);

void Motor_Stop(void);

void Motor_Forward(
    uint16_t leftPWM,
    uint16_t rightPWM
);

void Motor_Backward(
    uint16_t leftPWM,
    uint16_t rightPWM
);

void Motor_TurnLeft(void);
void Motor_TurnRight(void);
void Motor_UTurn(void);

void Nav_Step(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */


/* ============================================================
 * TCA9548A CHANNEL SELECT
 * ============================================================ */

uint8_t TCA_SelectChannel(uint8_t channel)
{
    if (HAL_I2C_Master_Transmit(
        &hi2c1,
        TCA9548A_ADDR,
        &channel,
        1,
        TCA_I2C_TIMEOUT_MS
    ) != HAL_OK)
    {
        return 0;
    }

    /*
     * Give the multiplexer a moment to switch.
     */
    HAL_Delay(1);
    return 1;
}


/* ============================================================
 * SENSOR FILTERING (median-of-3) + VALIDITY/OFFSET CORRECTION
 * ============================================================ */

static uint16_t median_of_3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

static uint16_t leftHistory[MEDIAN_FILTER_SAMPLES]  = {0};
static uint16_t frontHistory[MEDIAN_FILTER_SAMPLES] = {0};
static uint16_t rightHistory[MEDIAN_FILTER_SAMPLES] = {0};
static uint8_t leftHistoryCount, frontHistoryCount, rightHistoryCount;

static uint16_t filter_sample(uint16_t *hist, uint8_t *count, uint16_t value)
{
    hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = value;
    if (*count < MEDIAN_FILTER_SAMPLES) { (*count)++; }
    return (*count < MEDIAN_FILTER_SAMPLES) ? value : median_of_3(hist[0], hist[1], hist[2]);
}

/**
 * @brief Applies the out-of-range sentinel check, per-sensor calibration
 *        offset, and clamping to [0, MAX_VALID_DISTANCE_MM].
 */
static uint16_t apply_offset_and_clamp(uint16_t raw_mm, int16_t offset_mm)
{
    int32_t corrected;

    if (raw_mm >= SENSOR_TIMEOUT_SENTINEL_MM)
    {
        /* Out-of-range / timeout -> treat as fully open, not a real distance. */
        corrected = MAX_VALID_DISTANCE_MM;
    }
    else
    {
        corrected = (int32_t)raw_mm + offset_mm;
    }

    if (corrected < 0) corrected = 0;
    if (corrected > MAX_VALID_DISTANCE_MM) corrected = MAX_VALID_DISTANCE_MM;

    return (uint16_t)corrected;
}


/* ============================================================
 * SENSOR INITIALIZATION
 * ============================================================ */

uint8_t Sensors_Init(void)
{
    /*
     * ========================================================
     * LEFT SENSOR
     * TCA9548A CH0
     * ========================================================
     */

    if (!TCA_SelectChannel(TCA_LEFT_CHANNEL)) return 0;

    if (!initVL53L0X(1, &hi2c1))
    {
        return 0;
    }

    setSignalRateLimit(0.25);

    setVcselPulsePeriod(
        VcselPeriodPreRange,
        14
    );

    setVcselPulsePeriod(
        VcselPeriodFinalRange,
        10
    );

    setMeasurementTimingBudget(20000);


    /*
     * ========================================================
     * FRONT SENSOR
     * TCA9548A CH1
     * ========================================================
     */

    if (!TCA_SelectChannel(TCA_FRONT_CHANNEL)) return 0;

    if (!initVL53L0X(1, &hi2c1))
    {
        return 0;
    }

    setSignalRateLimit(0.25);

    setVcselPulsePeriod(
        VcselPeriodPreRange,
        14
    );

    setVcselPulsePeriod(
        VcselPeriodFinalRange,
        10
    );

    setMeasurementTimingBudget(20000);


    /*
     * ========================================================
     * RIGHT SENSOR
     * TCA9548A CH2
     * ========================================================
     */

    if (!TCA_SelectChannel(TCA_RIGHT_CHANNEL)) return 0;

    if (!initVL53L0X(1, &hi2c1))
    {
        return 0;
    }

    setSignalRateLimit(0.25);

    setVcselPulsePeriod(
        VcselPeriodPreRange,
        14
    );

    setVcselPulsePeriod(
        VcselPeriodFinalRange,
        10
    );

    setMeasurementTimingBudget(20000);


    /*
     * ========================================================
     * START CONTINUOUS MEASUREMENTS
     * ========================================================
     */

    setTimeout(30); /* must be finite: a disconnected sensor may not stall motion */
    TCA_SelectChannel(TCA_LEFT_CHANNEL);
    startContinuous(0);

    TCA_SelectChannel(TCA_FRONT_CHANNEL);
    startContinuous(0);

    TCA_SelectChannel(TCA_RIGHT_CHANNEL);
    startContinuous(0);


    return 1;
}


/* ============================================================
 * SENSOR READ FUNCTIONS (raw read -> offset/clamp -> median filter)
 * ============================================================ */

static SensorSample_t ReadSensor(uint8_t channel, statInfo_t_VL53L0X *stats,
                                 int16_t offset, uint16_t *history, uint8_t *historyCount)
{
    SensorSample_t sample = { MAX_VALID_DISTANCE_MM, SENSOR_I2C_ERROR };
    uint16_t raw;

    if (!TCA_SelectChannel(channel)) return sample;
    raw = readRangeContinuousMillimeters(stats);
    if (timeoutOccurred() || raw == 65535U) { sample.status = SENSOR_TIMEOUT; return sample; }
    if (stats->rangeStatus != 0U) { sample.status = SENSOR_RANGE_ERROR; return sample; }

    sample.mm = apply_offset_and_clamp(raw, offset);
    if (sample.mm >= MAX_VALID_DISTANCE_MM) {
        sample.status = SENSOR_FAR; /* valid measurement beyond navigation range */
    } else if (sample.mm < MIN_VALID_DISTANCE_MM) {
        /* Close readings remain valid: they are important for emergency stopping. */
        sample.status = SENSOR_OK;
    } else {
        sample.status = SENSOR_OK;
    }
    sample.mm = filter_sample(history, historyCount, sample.mm);
    return sample;
}

SensorSample_t ReadLeft(void)
{
    return ReadSensor(TCA_LEFT_CHANNEL, &leftStats, LEFT_SENSOR_OFFSET_MM, leftHistory, &leftHistoryCount);
}

SensorSample_t ReadFront(void)
{
    return ReadSensor(TCA_FRONT_CHANNEL, &frontStats, FRONT_SENSOR_OFFSET_MM, frontHistory, &frontHistoryCount);
}

SensorSample_t ReadRight(void)
{
    return ReadSensor(TCA_RIGHT_CHANNEL, &rightStats, RIGHT_SENSOR_OFFSET_MM, rightHistory, &rightHistoryCount);
}


void ReadAllSensors(void)
{
    leftSample = ReadLeft();   frontSample = ReadFront();   rightSample = ReadRight();
    leftDistance = leftSample.mm; frontDistance = frontSample.mm; rightDistance = rightSample.mm;
}


/* ============================================================
 * SENSOR STATE CLASSIFICATION (spec sections 4-5)
 * ============================================================ */

static inline uint8_t Front_IsOpen(uint16_t d)     { return d >= FRONT_OPEN_DISTANCE_MM; }
static inline uint8_t Front_IsBlocked(uint16_t d)  { return d <= FRONT_BLOCKED_DISTANCE_MM; }
static inline uint8_t Front_IsVeryOpen(uint16_t d) { return d >= MAX_VALID_DISTANCE_MM; }

static inline uint8_t Left_IsWallPresent(uint16_t d) { return d <= LEFT_WALL_PRESENT_THRESHOLD_MM; }
static inline uint8_t Left_IsOpen(uint16_t d)        { return d >= LEFT_OPEN_DISTANCE_MM; }

/* Right sensor reuses the same "open" threshold generically -- spec only
 * requires RIGHT for the front-blocked+left-blocked disambiguation case. */
static inline uint8_t Right_IsOpen(uint16_t d) { return d >= RIGHT_OPEN_DISTANCE_MM; }


/* ============================================================
 * MOTOR INITIALIZATION
 * ============================================================ */

void Motors_Init(void)
{
    /*
     * LEFT MOTOR
     *
     * PA0 -> TIM5_CH1 -> PWMA
     */

    HAL_TIM_PWM_Start(
        &htim5,
        TIM_CHANNEL_1
    );


    /*
     * RIGHT MOTOR
     *
     * PA5 -> TIM2_CH1 -> PWMB
     */

    HAL_TIM_PWM_Start(
        &htim2,
        TIM_CHANNEL_1
    );


    Motor_Stop();
}


/* ============================================================
 * MOTOR STOP
 * ============================================================ */

void Motor_Stop(void)
{
    /*
     * PWM = 0
     */

    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        0
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        0
    );


    /*
     * LEFT MOTOR
     *
     * IN1 = LOW
     * IN2 = LOW
     */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_1,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_2,
        GPIO_PIN_RESET
    );


    /*
     * RIGHT MOTOR
     *
     * IN1 = LOW
     * IN2 = LOW
     */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_3,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_4,
        GPIO_PIN_RESET
    );
}


/* ============================================================
 * DIRECTION HELPER (spec section 17: don't assume IN1 HIGH means
 * forward for both sides -- LEFT_MOTOR_INVERTED / RIGHT_MOTOR_INVERTED
 * are the single place that assumption gets corrected)
 * ============================================================ */

static void set_wheel_direction(GPIO_TypeDef *port, uint16_t pin_a, uint16_t pin_b,
                                 uint8_t forward, uint8_t inverted)
{
    uint8_t physicalForward = forward ^ inverted;

    if (physicalForward)
    {
        HAL_GPIO_WritePin(port, pin_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port, pin_b, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(port, pin_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port, pin_b, GPIO_PIN_SET);
    }
}


/* ============================================================
 * FORWARD
 * ============================================================ */

void Motor_Forward(
    uint16_t leftPWM,
    uint16_t rightPWM
)
{
    set_wheel_direction(GPIOA, GPIO_PIN_1, GPIO_PIN_2, 1, LEFT_MOTOR_INVERTED);
    set_wheel_direction(GPIOA, GPIO_PIN_3, GPIO_PIN_4, 1, RIGHT_MOTOR_INVERTED);

    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        leftPWM
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        rightPWM
    );
}


/* ============================================================
 * BACKWARD
 * ============================================================ */

void Motor_Backward(
    uint16_t leftPWM,
    uint16_t rightPWM
)
{
    set_wheel_direction(GPIOA, GPIO_PIN_1, GPIO_PIN_2, 0, LEFT_MOTOR_INVERTED);
    set_wheel_direction(GPIOA, GPIO_PIN_3, GPIO_PIN_4, 0, RIGHT_MOTOR_INVERTED);

    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        leftPWM
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        rightPWM
    );
}


/* ============================================================
 * TURN LEFT (in-place, counter-clockwise)
 * ============================================================ */

void Motor_TurnLeft(void)
{
    set_wheel_direction(GPIOA, GPIO_PIN_1, GPIO_PIN_2, 0, LEFT_MOTOR_INVERTED);  // left backward
    set_wheel_direction(GPIOA, GPIO_PIN_3, GPIO_PIN_4, 1, RIGHT_MOTOR_INVERTED); // right forward

    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        TURN_SPEED
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        TURN_SPEED
    );


    HAL_Delay(TURN_90_TIME_MS);

    Motor_Stop();
}


/* ============================================================
 * TURN RIGHT (in-place, clockwise)
 * ============================================================ */

void Motor_TurnRight(void)
{
    set_wheel_direction(GPIOA, GPIO_PIN_1, GPIO_PIN_2, 1, LEFT_MOTOR_INVERTED);  // left forward
    set_wheel_direction(GPIOA, GPIO_PIN_3, GPIO_PIN_4, 0, RIGHT_MOTOR_INVERTED); // right backward

    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        TURN_SPEED
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        TURN_SPEED
    );


    HAL_Delay(TURN_90_TIME_MS);

    Motor_Stop();
}


/* ============================================================
 * U-TURN (in-place, ~180 degrees)
 * ============================================================ */

void Motor_UTurn(void)
{
    set_wheel_direction(GPIOA, GPIO_PIN_1, GPIO_PIN_2, 0, LEFT_MOTOR_INVERTED);  // left backward
    set_wheel_direction(GPIOA, GPIO_PIN_3, GPIO_PIN_4, 1, RIGHT_MOTOR_INVERTED); // right forward

    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        TURN_SPEED
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        TURN_SPEED
    );


    HAL_Delay(TURN_180_TIME_MS);

    Motor_Stop();
}


/* ============================================================
 * LEFT-WALL FOLLOWER NAVIGATION (state machine, spec sections 15-18)
 * ============================================================ */

static NavState_t navState = NAV_FOLLOW_WALL;
static uint8_t leftOpenStreak = 0;
static uint8_t rightOpenStreak = 0;
static uint8_t frontBlockedStreak = 0;
static uint8_t exitConfirmStreak = 0;
static uint8_t sensorFaultStreak = 0;

static uint16_t clamp_pwm(int32_t value)
{
    if (value < (int32_t)MIN_SPEED) return MIN_SPEED;
    if (value > (int32_t)MAX_SPEED) return MAX_SPEED;
    return (uint16_t)value;
}

static void DriveCentered(void)
{
    int32_t error, correction;
    uint8_t leftWall = Left_IsWallPresent(leftDistance);
    uint8_t rightWall = rightDistance <= LEFT_WALL_PRESENT_THRESHOLD_MM;

    if (leftWall && rightWall) error = (int32_t)leftDistance - (int32_t)rightDistance;
    else if (leftWall) error = (int32_t)leftDistance - SIDE_TARGET_DISTANCE_MM;
    else if (rightWall) error = SIDE_TARGET_DISTANCE_MM - (int32_t)rightDistance;
    else { Motor_Forward(BASE_SPEED, BASE_SPEED); return; }

    if (error >= -WALL_DEADBAND_MM && error <= WALL_DEADBAND_MM) correction = 0;
    else correction = (error * WALL_KP_NUMERATOR) / WALL_KP_DENOMINATOR;
    if (correction > MAX_WALL_CORRECTION) correction = MAX_WALL_CORRECTION;
    if (correction < -MAX_WALL_CORRECTION) correction = -MAX_WALL_CORRECTION;
    correction *= WALL_STEERING_SIGN;
    Motor_Forward(clamp_pwm((int32_t)BASE_SPEED - correction),
                  clamp_pwm((int32_t)BASE_SPEED + correction));
}

/* Enable DEBUG_MODE only after retargeting printf to a UART or SWO in CubeIDE.
 * No UART is assigned in this .ioc, so this is compiled out by default. */
static void Debug_Report(const char *action)
{
#if DEBUG_MODE
    printf("L:%u(%u) F:%u(%u) R:%u(%u) LW:%s FO:%s RO:%s CENTER:%s ACTION:%s\r\n",
           leftDistance, leftSample.status, frontDistance, frontSample.status,
           rightDistance, rightSample.status,
           Left_IsWallPresent(leftDistance) ? "YES" : "NO",
           Front_IsOpen(frontDistance) ? "OPEN" : "BLOCKED",
           Right_IsOpen(rightDistance) ? "OPEN" : "WALL",
           (Left_IsWallPresent(leftDistance) || rightDistance <= LEFT_WALL_PRESENT_THRESHOLD_MM) ? "ON" : "OFF",
           action);
#else
    (void)action;
#endif
}

void Nav_Step(void)
{
    if (navState == NAV_STOPPED)
    {
        return; // Maze finished. Permanently idle.
    }

    ReadAllSensors();

    /* An invalid sample is never an opening. Stop after repeated communication
     * failures rather than navigating from invented geometry. SENSOR_FAR is a
     * valid, deliberately open reading. */
    if (leftSample.status >= SENSOR_TIMEOUT || frontSample.status >= SENSOR_TIMEOUT ||
        rightSample.status >= SENSOR_TIMEOUT)
    {
        if (++sensorFaultStreak >= SENSOR_FAULT_CONFIRM_COUNT) {
            Debug_Report("SENSOR FAULT STOP");
            Motor_Stop(); navState = NAV_STOPPED; return;
        }
        Motor_Stop();
        return;
    }
    sensorFaultStreak = 0;

    uint8_t frontVeryOpen   = Front_IsVeryOpen(frontDistance);
    uint8_t frontOpen       = Front_IsOpen(frontDistance);
    uint8_t leftWallPresent = Left_IsWallPresent(leftDistance);
    uint8_t leftOpen        = Left_IsOpen(leftDistance);

    /*
     * ========================================================
     * PRIORITY 1: EXIT DETECTION (highest priority -- spec section 13)
     *
     * LEFT_OPEN and LEFT_WALL_PRESENT are mutually exclusive by
     * definition (leftDistance is either > or <= the threshold), so
     * there is no real ambiguity between this and an ordinary left
     * intersection -- checking exit first is still done explicitly
     * per spec for clarity and robustness.
     * ========================================================
     */

    if (frontVeryOpen && frontSample.status == SENSOR_FAR && leftWallPresent)
    {
        exitConfirmStreak++;
    }
    else
    {
        exitConfirmStreak = 0;
    }

    if (exitConfirmStreak >= EXIT_CONFIRMATION_COUNT)
    {
        Debug_Report("EXIT STOP");
        navState = NAV_EXIT_DETECTED;
        Motor_Stop();
        navState = NAV_STOPPED;
        return;
    }

    /*
     * ========================================================
     * PRIORITY 2: LEFT PATH
     * ========================================================
     */

    if (leftOpen && leftSample.status != SENSOR_RANGE_ERROR)
    {
        leftOpenStreak++;
    }
    else
    {
        leftOpenStreak = 0;
    }

    if (leftOpenStreak >= LEFT_OPEN_CONFIRM_COUNT)
    {
        Debug_Report("LEFT TURN");
        navState = NAV_APPROACH_LEFT_TURN;

        Motor_Forward(BASE_SPEED, BASE_SPEED);
        HAL_Delay(APPROACH_CREEP_MS);

        Motor_Stop();
        HAL_Delay(STOP_PAUSE_MS);

        navState = NAV_TURN_LEFT;
        Motor_TurnLeft();

        leftOpenStreak = 0;
        navState = NAV_FOLLOW_WALL;
        return;
    }

    /*
     * ========================================================
     * PRIORITY 3: FRONT PATH -- wall-following with differential correction
     * ========================================================
     */

    if (Front_IsBlocked(frontDistance)) frontBlockedStreak++; else frontBlockedStreak = 0;
    if (rightDistance >= RIGHT_OPEN_DISTANCE_MM) rightOpenStreak++; else rightOpenStreak = 0;

    if (frontOpen)
    {
        navState = NAV_FOLLOW_WALL;
        Debug_Report("FORWARD");
        DriveCentered();

        return;
    }

    /*
     * ========================================================
     * PRIORITY 4: FRONT BLOCKED + LEFT BLOCKED
     *
     * Use RIGHT sensor to disambiguate a right turn from a dead end
     * (spec section 10, preferred implementation).
     * ========================================================
     */

    if (frontBlockedStreak < FRONT_BLOCKED_CONFIRM_COUNT) {
        Motor_Stop(); /* hysteresis region or first blocked sample */
        return;
    }
    Motor_Stop();
    HAL_Delay(STOP_PAUSE_MS);

    if (rightOpenStreak >= RIGHT_OPEN_CONFIRM_COUNT)
    {
        Debug_Report("RIGHT TURN");
        navState = NAV_TURN_RIGHT;
        Motor_TurnRight();
    }
    else
    {
        Debug_Report("U TURN");
        navState = NAV_U_TURN;
        Motor_UTurn();
    }

    navState = NAV_FOLLOW_WALL;
}


/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--------------------------------------------------------*/

    /*
     * Reset of all peripherals, Initializes the Flash interface
     * and the Systick.
     */

    HAL_Init();


    /* USER CODE BEGIN Init */

    /* USER CODE END Init */


    /*
     * Configure the system clock
     */

    SystemClock_Config();


    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */


    /*
     * Initialize all configured peripherals
     */

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM5_Init();


    /* USER CODE BEGIN 2 */


    /*
     * ========================================================
     * INITIALIZE MOTORS
     * ========================================================
     */

    Motors_Init();


    /*
     * ========================================================
     * INITIALIZE SENSORS
     * ========================================================
     */

    if (!Sensors_Init())
    {
        /*
         * Sensor initialization failed.
         *
         * Stop motors.
         *
         * Blink PC13 rapidly forever.
         */

        Motor_Stop();

        while (1)
        {
            HAL_GPIO_TogglePin(
                GPIOC,
                GPIO_PIN_13
            );

            HAL_Delay(150);
        }
    }


    /*
     * ========================================================
     * SENSOR INITIALIZATION SUCCESS
     * ========================================================
     *
     * One slow blink indicates that all three sensors
     * initialized successfully.
     */

    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_13,
        GPIO_PIN_RESET
    );

    HAL_Delay(500);

    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_13,
        GPIO_PIN_SET
    );

    HAL_Delay(500);


    /* USER CODE END 2 */


    /* Infinite loop */

    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /*
         * Execute one navigation control cycle.
         */

        Nav_Step();


        /*
         * Small delay between control iterations.
         */

        HAL_Delay(10);
    }

    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /*
     * Configure the main internal regulator output voltage
     */

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE2
    );


    /*
     * Initializes the RCC Oscillators according to the specified
     * parameters in the RCC_OscInitTypeDef structure.
     */

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 8;

    RCC_OscInitStruct.PLL.PLLN = 84;

    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ = 4;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Initializes the CPU, AHB and APB buses clocks
     */

    RCC_ClkInitStruct.ClockType =
          RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1
        | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2
        ) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */

static void MX_I2C1_Init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */


    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */


    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = 100000;

    hi2c1.Init.DutyCycle =
        I2C_DUTYCYCLE_2;

    hi2c1.Init.OwnAddress1 = 0;

    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c1.Init.OwnAddress2 = 0;

    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM2_Init(void)
{
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */


    TIM_ClockConfigTypeDef sClockSourceConfig = {0};

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    TIM_OC_InitTypeDef sConfigOC = {0};


    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */


    htim2.Instance = TIM2;

    htim2.Init.Prescaler = 83;

    htim2.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim2.Init.Period = 49;

    htim2.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    htim2.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }


    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (HAL_TIM_ConfigClockSource(
            &htim2,
            &sClockSourceConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim2,
            &sMasterConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 0;

    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    if (HAL_TIM_PWM_ConfigChannel(
            &htim2,
            &sConfigOC,
            TIM_CHANNEL_1
        ) != HAL_OK)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */


    HAL_TIM_MspPostInit(&htim2);
}


/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM5_Init(void)
{
    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */


    TIM_ClockConfigTypeDef sClockSourceConfig = {0};

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    TIM_OC_InitTypeDef sConfigOC = {0};


    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */


    htim5.Instance = TIM5;

    /*
     * Same PWM timing as TIM2.
     *
     * Timer clock = 84 MHz
     * Prescaler   = 83
     * Timer clock = 1 MHz
     *
     * Period = 49
     * PWM frequency = 1 MHz / 50 = 20 kHz
     */

    htim5.Init.Prescaler = 83;

    htim5.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim5.Init.Period = 49;

    htim5.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    htim5.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }


    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (HAL_TIM_ConfigClockSource(
            &htim5,
            &sClockSourceConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim5,
            &sMasterConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 0;

    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    if (HAL_TIM_PWM_ConfigChannel(
            &htim5,
            &sConfigOC,
            TIM_CHANNEL_1
        ) != HAL_OK)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */


    HAL_TIM_MspPostInit(&htim5);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */


    /* GPIO Ports Clock Enable */

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();


    /* Configure GPIO pin Output Level */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_1 |
        GPIO_PIN_2 |
        GPIO_PIN_3 |
        GPIO_PIN_4,
        GPIO_PIN_RESET
    );


    /* Configure GPIO pins : PA1 PA2 PA3 PA4 */

    GPIO_InitStruct.Pin =
          GPIO_PIN_1
        | GPIO_PIN_2
        | GPIO_PIN_3
        | GPIO_PIN_4;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* PB12 mode/start button: internally pulled up, active low. */

    GPIO_InitStruct.Pin =
        GPIO_PIN_12;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_PULLUP;


    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );

    /* PC13 onboard LED is active-low. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

/*
 * No UART debugging is used.
 *
 * PC13 is used only as the diagnostic LED:
 *
 *   Rapid blinking = sensor initialization failure
 *   One startup blink = sensor initialization successful
 */

/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    /*
     * User can add his own implementation to report
     * the HAL error return state.
     */

    __disable_irq();

    while (1)
    {
    }

    /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */

void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */

    /*
     * User can add his own implementation to report the file name
     * and line number, ex: printf("Wrong parameters value: file %s
     * on line %d\r\n", file, line)
     */

    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
