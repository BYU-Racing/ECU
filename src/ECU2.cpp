#include "ECU2.h"
#include "Reserved.h"

ECU::ECU()
{
    throttle = Throttle();
    brake = Brake();
}

ECU::~ECU()
{
    shutdown();
}

/** Used to set communication ports, output pins, and get initial state */
void ECU::begin(const FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& motorCAN,
               const FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& dataCAN)
{
    this->motorCAN = motorCAN;
    this->dataCAN = dataCAN;
    pinMode(BL_PIN, OUTPUT);
    pinMode(HORN_PIN, OUTPUT);
    updateHealth();
}

/** Send a health check can request to all DCs */
void ECU::sendHealthCheckRequest()
{
    outMsg.id = HealthCheckId;
    outMsg.len = 0;
    dataCAN.write(outMsg);
}

/** Send and await health check from all DCs and update internal health */
void ECU::updateHealth()
{
    sendHealthCheckRequest();
    Health worstHealth = HEALTHY; // Assume all sensors are healthy
    const uint32_t timer = millis();
    while (millis() - timer <= 150) // Wait 150 ms for responses
    {
        // TODO: Since we aren't using mailboxes, could this miss responses while processing?
        if (dataCAN.read(inMsg))
        {
            switch (inMsg.id)
            {
            case DCFId:
            case DCRId:
            case DCTId:
                // Each response is an array of all sensor healths, not just a single one
                // Get the worst sensor health of
                for (uint8_t i = 0; i < inMsg.len; i++)
                {
                    if (const Health health = static_cast<Health>(inMsg.buf[i]); health < worstHealth)
                    {
                        worstHealth = health;
                    }
                }
                break;
            default:
                break;
            }
        }
    }
    health = worstHealth;
    lastHealthUpdate = timer;
}

/** Initiate the startup sequence, if all conditions are met */
void ECU::startup()
{
    digitalWrite(HORN_PIN, HIGH);
    delay(2000); // Delay for 2 seconds per rules
    digitalWrite(HORN_PIN, LOW);
    driveState = true;
    BTOverride = false;
    outMsg.id = DriveStateId;
    outMsg.len = 1;
    outMsg.buf[0] = driveState;
    dataCAN.write(outMsg);
    enableInverter();
}

/** Initiate the shutdown sequence */
void ECU::shutdown()
{
    driveState = false;
    outMsg.id = DriveStateId;
    outMsg.len = 1;
    outMsg.buf[0] = driveState;
    dataCAN.write(outMsg);
    disableInverter();
}

/** Main operation entrypoint for infinite looping */
void ECU::run()
{
    if (!driveState)
    {
        // TODO: Should this send a notice to the driver?
        attemptStartup();
    }
    // IMPORTANT: Reads must be separate to not
    // skip the second call if the first returned true
    if (motorCAN.read(inMsg))
    {
        routeMsg();
    }
    if (dataCAN.read(inMsg))
    {
        routeMsg();
    }
    if (millis() - lastHealthUpdate <= 3000)
    {
        updateHealth();
    }
    if (health == CRITICAL || health == UNKNOWN)
    {
        shutdown();
    }
}

void ECU::attemptStartup()
{
    if (inverterLockout == LOCKED)
    {
        // Inverter is locked because of a fault, try unlocking
        disableInverter();
    }
    else if (brake.getBrakeActive() && !startFault && tractiveActive && startSwitch)
    {
        // All start conditions are met - begin startup sequence
        startup();
    }
    else if (prevStartSwitch && !startSwitch && startFault)
    {
        // We were in start fault but turned the switch off
        startFault = false;
    }
    else if (startSwitch && !prevStartSwitch)
    {
        // We did not meet conditions but turned the switch on
        startFault = true;
        throwError(StartFaultId);
    }
}


/** Routes the inMsg (by id) to the appropriate update function */
void ECU::routeMsg()
{
    switch (inMsg.id)
    {
    case Throttle1PositionId:
        updateThrottle();
        break;
    case Throttle2PositionId:
        updateThrottle();
        break;
    case BrakePressureId:
        updateBrake();
        break;
    case StartSwitchId:
        updateStartSwitch();
        break;
    case ThrottleMinId:
        calibrateThrottleMin();
        break;
    case ThrottleMaxId:
        calibrateThrottleMax();
        break;
    case DriveModeId:
        updateDriveMode();
        break;
    case InternalStatesId:
        updateInverter();
        break;
    default:
        break;
    }
}

void ECU::updateThrottle()
{
    unpacker.reset(inMsg.buf);
    if (inMsg.id == Throttle1PositionId)
    {
        throttle.setThrottle1(unpacker.unpack<int32_t>());
        updatedThrottle1 = true;
    }
    else
    {
        throttle.setThrottle2(unpacker.unpack<int32_t>());
        updatedThrottle2 = true;
    }
    if (!updatedThrottle1 || !updatedThrottle2)
    {
        return;
    }
    if (const int throttleCode = throttle.checkError(); throttleCode != 0)
    {
        throwError(throttleCode);
        return;
    }
    updatedThrottle1 = false;
    updatedThrottle2 = false;
    motorCommand(throttle.calculateTorque());
}

void ECU::calibrateThrottleMin()
{
    unpacker.reset(inMsg.buf);
    const int32_t throttleMin = unpacker.unpack<int32_t>();
    throttle.setCalibrationValueMin(throttleMin, throttleMin);
}

void ECU::calibrateThrottleMax()
{
    unpacker.reset(inMsg.buf);
    const int32_t throttleMax = unpacker.unpack<int32_t>();
    throttle.setCalibrationValueMin(throttleMax, throttleMax);
}

void ECU::updateBrake()
{
    unpacker.reset(inMsg.buf);
    brake.updateValue(unpacker.unpack<int32_t>());
    if (brake.getBrakeErrorState() == 2)
    {
        throwError(BrakeZeroId);
    }
}

void ECU::updateStartSwitch()
{
    prevStartSwitch = startSwitch;
    startSwitch = inMsg.buf[0] == 1;
    if (!startSwitch && driveState)
    {
        // Turning off the switch shuts down the car
        shutdown();
    }
}

void ECU::updateDriveMode()
{
#define MAX_SPEED_ADDRESS 128
    if (inMsg.buf[0] == FULL_BEANS && driveMode != FULL_BEANS)
    {
        outMsg.id = ParameterCommandId;
        outMsg.len = 8;
        outMsg.buf[0] = MAX_SPEED_ADDRESS;
        outMsg.buf[1] = 0;
        outMsg.buf[2] = 1; // Write
        outMsg.buf[3] = 0; // Unused
        // Max RPM
        outMsg.buf[4] = 255;
        outMsg.buf[5] = 255;
        outMsg.buf[6] = 0; // Unused
        outMsg.buf[7] = 0; // Unused
        motorCAN.write(outMsg);
        driveMode = FULL_BEANS;
        throttle.setMaxTorque(3100);
    }
    else if (inMsg.buf[0] == ENDURANCE && driveMode != ENDURANCE)
    {
        outMsg.id = ParameterCommandId;
        outMsg.len = 8;
        outMsg.buf[0] = MAX_SPEED_ADDRESS;
        outMsg.buf[1] = 0;
        outMsg.buf[2] = 1; // Write
        outMsg.buf[3] = 0; // Unused
        // Max RPM
        outMsg.buf[4] = 255;
        outMsg.buf[5] = 255;
        outMsg.buf[6] = 0; // Unused
        outMsg.buf[7] = 0; // Unused
        motorCAN.write(outMsg);
        driveMode = ENDURANCE;
        throttle.setMaxTorque(1500);
    }
    else if (inMsg.buf[0] == SKID_PAD && driveMode != SKID_PAD)
    {
        // TODO: Figure out why this is the same as ENDURANCE
        outMsg.id = ParameterCommandId;
        outMsg.len = 8;
        outMsg.buf[0] = MAX_SPEED_ADDRESS;
        outMsg.buf[1] = 0;
        outMsg.buf[2] = 1; // Write
        outMsg.buf[3] = 0; // Unused
        // Max RPM
        outMsg.buf[4] = 255;
        outMsg.buf[5] = 255;
        outMsg.buf[6] = 0; // Unused
        outMsg.buf[7] = 0; // Unused
        motorCAN.write(outMsg);
        driveMode = SKID_PAD;
        throttle.setMaxTorque(1500);
    }
}


void ECU::updateInverter()
{
    // Cascadia Motion CM200DX Software docs:
    // https://www.cascadiamotion.com/uploads/5/1/3/0/51309945/0a-0163-02_sw_user_manual.pdf
    // Page 144-147 for msg id 0x0AA
    unpacker.reset(inMsg.buf);
    vsmState = unpacker.unpack<VSMState>();
    unpacker.skip<uint16_t>();
    const uint8_t Relays = unpacker.unpack<uint8_t>();
    prechargeRelay = Relays & 0b1;
    mainRelay = Relays & 0b10;
    unpacker.skip<uint16_t>();
    const uint8_t Enabled = unpacker.unpack<uint8_t>();
    inverterEnabled = Enabled & 0b1;
    inverterLockout = static_cast<INV_Lockout>(Enabled & 0b10);
    tractiveActive = vsmState == WAIT && prechargeRelay && mainRelay && inverterLockout == UNLOCKED;
}

/** Sends the command to enable the inverter. A check for tractiveActive is assumed to have been done before calling */
void ECU::enableInverter()
{
    // Only enable the inverter when in VSM 4 and it has passed precharge
    outMsg.id = ControlCommandId;
    outMsg.len = 8;
    // Ignore torque
    outMsg.buf[0] = 0;
    outMsg.buf[1] = 0;
    // Ignore speed
    outMsg.buf[2] = 0;
    outMsg.buf[3] = 0;
    // Command forward
    outMsg.buf[4] = FORWARD;
    // Set enable bit
    outMsg.buf[5] = ENABLED;
    // Ignore torque limit
    outMsg.buf[6] = 0;
    outMsg.buf[7] = 0;
    motorCAN.write(outMsg);
}

void ECU::disableInverter()
{
    outMsg.id = ControlCommandId;
    outMsg.len = 8;
    // Ignore torque
    outMsg.buf[0] = 0;
    outMsg.buf[1] = 0;
    // Ignore speed
    outMsg.buf[2] = 0;
    outMsg.buf[3] = 0;
    // Command forward
    outMsg.buf[4] = FORWARD;
    // Set enable bit to disable
    outMsg.buf[5] = DISABLED;
    // Ignore torque limit
    outMsg.buf[6] = 0;
    outMsg.buf[7] = 0;
    motorCAN.write(outMsg);
}

void ECU::motorCommand(const int torque)
{
    if (tractiveActive && !inverterEnabled)
    {
        enableInverter();
    }
    if (driveState)
    {
        updateBTOverride(torque);
    }
    if (tractiveActive && driveState && !BTOverride)
    {
        outMsg.id = ControlCommandId;
        outMsg.len = 8;
        // Specify torque
        outMsg.buf[0] = torque % 256;
        outMsg.buf[1] = torque / 256;
        // Ignore Speed
        outMsg.buf[2] = 0;
        outMsg.buf[3] = 0;
        // Command forward
        outMsg.buf[4] = FORWARD;
        // Reaffirm enable bit
        outMsg.buf[5] = ENABLED;
        // Ignore torque limit
        outMsg.buf[6] = 0;
        outMsg.buf[7] = 0;
        motorCAN.write(outMsg);
    } else if (tractiveActive && !driveState)
    {
        // Stop the motor from spinning with a 0-torque command, which is identical to the "empty" enable command
        enableInverter();
    }
}

void ECU::updateBTOverride(const int torque)
{
    if (BTOverride && !brake.getBrakeActive() && torque <= BTO_OFF_THRESHOLD)
    {
        BTOverride = false;
    }
    if (torque >= BTO_ON_THRESHOLD && !BTOverride && brake.getBrakeActive())
    {
        BTOverride = true;
    }
}

/** Throw an error to the data CAN bus and shutdown */
void ECU::throwError(const uint8_t code)
{
    outMsg.id = FaultId;
    outMsg.len = 1;
    outMsg.buf[0] = code;
    dataCAN.write(outMsg);
    shutdown(); // Shutdown to trigger a change to driveState
}
