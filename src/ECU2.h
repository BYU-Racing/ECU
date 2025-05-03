#ifndef ECU2_H
#define ECU2_H

#include "FlexCAN_T4.h"
#include "Throttle.h"
#include "Brake.h"
#include "BufferPacker.h"

typedef enum VSM_States : uint8_t
{
    START,
    PRECHARGE_INIT,
    PRECHARGE_ACTIVE,
    PRECHARGE_COMPLETE,
    WAIT,
    READY,
    MOTOR_RUNNING,
    FAULT,
    SHUTDOWN_IN_PROCESS = 14,
    RECYCLE_POWER
} VSMState;

typedef enum INV_Lockouts : uint8_t
{
    UNLOCKED,
    LOCKED,
} INV_Lockout;

typedef enum INV_Enable : uint8_t
{
    DISABLED,
    ENABLED
} INV_Enable;

typedef enum Directions : uint8_t
{
    REVERSE,
    FORWARD
} Direction;

typedef enum DC_Healths : uint8_t
{
    /** Health cannot be determined; use for default initializations */
    UNKNOWN,
    /** A critical sensor is unresponsive */
    CRITICAL,
    /** A non-critical sensor is unresponsive */
    UNRESPONSIVE,
    /** Limited operating functionality */
    DEGRADED,
    /** Full operating functionality */
    HEALTHY,
} Health;

typedef enum Drive_Modes : uint8_t
{
    FULL_BEANS,
    ENDURANCE,
    SKID_PAD,
} DriveMode;

constexpr int HORN_PIN = 15;
constexpr int BL_PIN = 16;
constexpr int BTO_OFF_THRESHOLD = 120;
constexpr int BTO_ON_THRESHOLD = 300;

class ECU
{
public:
    ECU();
    ~ECU();
    void begin(const FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& motorCAN,
              const FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& dataCAN);
    void run();

private:
    /// Communications

    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> dataCAN;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCAN;
    CAN_message_t outMsg;
    CAN_message_t inMsg;
    BufferPacker<8> unpacker;
    void routeMsg();
    // void routeMsg(const CAN_message_t &msg); // Maybe a mailbox approach like BB?

    /// State management

    // Used for tracking the order of start procedure
    bool startFault = false;
    // Used for determining whether the car can actively drive
    bool driveState = false;
    DriveMode driveMode = FULL_BEANS;
    void updateDriveMode();

    VSMState vsmState = START;
    // AIR+
    bool prechargeRelay = false;
    // AIR-
    bool mainRelay = false;
    bool inverterEnabled = false;
    // If inverter requires disabling before enabling again
    INV_Lockouts inverterLockout = UNLOCKED;
    bool tractiveActive = false;
    void updateInverter();

    bool startSwitch = false;
    bool prevStartSwitch = false;
    void updateStartSwitch();

    Brake brake;
    void updateBrake();

    Throttle throttle;
    bool updatedThrottle1 = false;
    bool updatedThrottle2 = false;
    void updateThrottle();
    void calibrateThrottleMin();
    void calibrateThrottleMax();

    // Refers to the overall of all sensors polled by a data collector health check
    Health health = HEALTHY;
    uint32_t lastHealthUpdate = 0;
    void sendHealthCheckRequest();
    void updateHealth();

    bool BTOverride = false;
    void updateBTOverride(int torque);

    /// Actions

    void attemptStartup();
    void startup();
    void shutdown();

    void enableInverter();
    void disableInverter();
    void motorCommand(int torque);

    void throwError(uint8_t code);
};

#endif //ECU2_H
