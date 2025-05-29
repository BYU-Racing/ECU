#include "ECU.h"
#include "Reserved.h"

ECU::ECU()
{
    // Serial.println("[ECU] Constructor");
    throttle = Throttle();
    brake = Brake();
}

ECU::~ECU()
{
    // Serial.println("[ECU] Destructor: shutting down");
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
    // Serial.println("[ECU] begin() complete");
}

/** Send a health check CAN request to all DCs */
void ECU::sendHealthCheckRequest()
{
    // Serial.println("[ECU] sendHealthCheckRequest");
    outMsg.id = HealthCheckId;
    outMsg.len = 0;
    dataCAN.write(outMsg);
}

/** Send and await health check from all DCs and update internal health */
void ECU::updateHealth()
{
    // Serial.println("[ECU] updateHealth()");
    sendHealthCheckRequest();
    Health worstHealth = HEALTHY; // Assume all sensors are healthy
    const uint32_t timer = millis();
    // Serial.print("[ECU]   waiting responses until ");
    // Serial.println(timer + 150);
    while (millis() - timer <= 150) // Wait 150 ms for responses
    {
        if (dataCAN.read(inMsg))
        {
            // Serial.print("[ECU]   got CAN id=0x");
            // Serial.println(inMsg.id, HEX);
            switch (inMsg.id)
            {
            case DCFId:
            case DCRId:
            case DCTId:
                for (uint8_t i = 0; i < inMsg.len; i++)
                {
                    Health h = static_cast<Health>(inMsg.buf[i]);
                    if (h < worstHealth)
                    {
                        // Serial.print("[ECU]     new worstHealth=");
                        // Serial.println((uint8_t)h);
                        worstHealth = h;
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
    // Serial.print("[ECU]   final health=");
    // Serial.println((uint8_t)health);
}

/** Initiate the startup sequence, if all conditions are met */
void ECU::startup()
{
    // Serial.println("[ECU] startup()");
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
    // Serial.println("[ECU] shutdown()");
    driveState = false;
    outMsg.id   = DriveStateId;
    outMsg.len  = 1;
    outMsg.buf[0] = driveState;
    dataCAN.write(outMsg);
    disableInverter();
}

/** Main operation entrypoint for infinite looping */
void ECU::run()
{
    // Rate-limited serial prints
    static uint32_t lastRunSerial = 0;
    const uint32_t now = millis();
    const uint32_t RUN_PRINT_INTERVAL = 250; // ms
    bool doPrint = (now - lastRunSerial >= RUN_PRINT_INTERVAL);
    if (doPrint) {
        lastRunSerial = now;
        Serial.print("[ECU] run() driveState=");
        Serial.print(driveState);
        Serial.print(" health=");
        Serial.print((uint8_t)health);
        Serial.print(" dt=");
        Serial.println(now - lastHealthUpdate);
    }

    if (!driveState)
    {
        if (doPrint) Serial.println("[ECU]   driveState false -> attemptStartup");
        attemptStartup();
    }

    // IMPORTANT: Reads must be separate to not skip the second call if the first returned true
    if (motorCAN.read(inMsg))
    {
        if (doPrint) {
            Serial.print("[ECU]   motorCAN msg id=0x");
            Serial.println(inMsg.id, HEX);
        }
        routeMsg();
    }
    if (dataCAN.read(inMsg))
    {
        if (doPrint) {
            Serial.print("[ECU]   dataCAN msg id=0x");
            Serial.println(inMsg.id, HEX);
        }
        routeMsg();
    }

    if (now - lastHealthUpdate >= 3000)
    {
        if (doPrint) Serial.println("[ECU]   health stale -> updateHealth");
        updateHealth();
    }

    if (health == CRITICAL || health == UNKNOWN)
    {
        if (doPrint) Serial.println("[ECU]   health fault -> shutdown");
        // shutdown(); // Health is not being considered a fault right now
    }
}

void ECU::attemptStartup()
{
    // Rate-limited serial prints
    static uint32_t lastAttemptStartSerial = 0;
    const uint32_t now = millis();
    const uint32_t ATTEMPT_STARTUP_PRINT_INTERVAL = 250; // ms
    bool doPrint = (now - lastAttemptStartSerial >= ATTEMPT_STARTUP_PRINT_INTERVAL);
    if (doPrint)
    {
        lastAttemptStartSerial = now;
        Serial.print("[ECU] attemptStartup: lockout=");
        Serial.print((uint8_t)inverterLockout);
        Serial.print(" brake=");
        Serial.print(brake.getBrakeActive());
        Serial.print(" startFault=");
        Serial.print(startFault);
        Serial.print(" tractiveActive=");
        Serial.print(tractiveActive);
        Serial.print(" startSwitch=");
        Serial.println(startSwitch);
    }
    if (inverterLockout == LOCKED)
    {
        // Inverter is locked because of a fault, try unlocking
        if (doPrint) Serial.println("[ECU]   inverter locked -> disableInverter");
        disableInverter();
    }
    else if (!startFault && startSwitch) // Used while brake is not pressurized
    // else if (brake.getBrakeActive() && !startFault && startSwitch) // Commented out when brake is not pressurized
    {
        // All start conditions are met - begin startup sequence
        if (doPrint) Serial.println("[ECU]   conditions met -> startup");
        startup();
    }
    else if (prevStartSwitch && !startSwitch && startFault)
    {
        // We were in start fault but turned the switch off
        if (doPrint) Serial.println("[ECU]   clearing startFault");
        startFault = false;
    }
    else if (startSwitch && !prevStartSwitch)
    {
        // We did not meet conditions but turned the switch on
        if (doPrint) Serial.println("[ECU]   bad startSwitch -> throwError");
        startFault = true;
        throwError(StartFaultId);
    }
}

/** Routes the inMsg (by id) to the appropriate update function */
void ECU::routeMsg()
{
    // Serial.print("[ECU] routeMsg id=0x");
    // Serial.println(inMsg.id, HEX);
    switch (inMsg.id)
    {
    case Throttle1PositionId:
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
        // Serial.println("[ECU]   unknown CAN id");
        break;
    }
}

void ECU::updateThrottle()
{
    // Serial.println("[ECU] updateThrottle");
    unpacker.reset(inMsg.buf);
    if (inMsg.id == Throttle1PositionId)
    {
        int32_t val = unpacker.unpack<int32_t>();
        throttle.setThrottle1(val);
        // Serial.print("[ECU]   throttle1="); Serial.println(val);
        updatedThrottle1 = true;
    }
    else
    {
        int32_t val = unpacker.unpack<int32_t>();
        throttle.setThrottle2(val);
        // Serial.print("[ECU]   throttle2="); Serial.println(val);
        updatedThrottle2 = true;
    }
    if (!updatedThrottle1 || !updatedThrottle2)
    {
        return;
    }

    int code = throttle.checkError();
    if (code != 0)
    {
        throwError(code);
        return;
    }
    updatedThrottle1 = updatedThrottle2 = false;
    int torque = throttle.calculateTorque();
    Serial.print("[ECU]   calculated torque="); Serial.println(torque);
    motorCommand(torque);
}

void ECU::calibrateThrottleMin()
{
    // Serial.println("[ECU] calibrateThrottleMin");
    unpacker.reset(inMsg.buf);
    int32_t val = unpacker.unpack<int32_t>();
    // Serial.print("[ECU]   min="); Serial.println(val);
    throttle.setCalibrationValueMin(val, val);
}

void ECU::calibrateThrottleMax()
{
    // Serial.println("[ECU] calibrateThrottleMax");
    unpacker.reset(inMsg.buf);
    int32_t val = unpacker.unpack<int32_t>();
    // Serial.print("[ECU]   max="); Serial.println(val);
    throttle.setCalibrationValueMin(val, val);
}

void ECU::updateBrake()
{
    // Serial.println("[ECU] updateBrake");
    unpacker.reset(inMsg.buf);
    int32_t val = unpacker.unpack<int32_t>();
    // Serial.print("[ECU]   brakePressure="); Serial.println(val);
    brake.updateValue(val);
    if (brake.getBrakeErrorState() == 2)
    {
        // Serial.println("[ECU]   brake zero error");
        throwError(BrakeZeroId);
    }
}

void ECU::updateStartSwitch()
{
    // Serial.println("[ECU] updateStartSwitch");
    prevStartSwitch = startSwitch;
    startSwitch = (inMsg.buf[0] == 1);
    // Serial.print("[ECU]   startSwitch="); Serial.println(startSwitch);
    if (!startSwitch && driveState)
    {
        // Turning off the switch shuts down the car
        Serial.println("[ECU]   switch off -> shutdown");
        shutdown();
    }
}

void ECU::updateDriveMode()
{
    Serial.println("[ECU] updateDriveMode");
    uint8_t req = inMsg.buf[0];
    Serial.print("[ECU]   req="); Serial.print(req);
    Serial.print(" current="); Serial.println(driveMode);
    auto sendParam = [&]() {
        outMsg.id = ParameterCommandId;
        outMsg.len = 8;
        outMsg.buf[0] = 128; // MAX_SPEED_ADDRESS
        outMsg.buf[1] = 0;
        outMsg.buf[2] = 1; // Write
        outMsg.buf[3] = 0; // Unused
        // Max RPM
        outMsg.buf[4] = 255;
        outMsg.buf[5] = 255;
        outMsg.buf[6] = 0; // Unused
        outMsg.buf[7] = 0; // Unused
        motorCAN.write(outMsg);
    };
    if (req == FULL_BEANS && driveMode != FULL_BEANS)
    {
        Serial.println("[ECU]   switching to FULL_BEANS");
        sendParam();
        driveMode = FULL_BEANS;
        throttle.setMaxTorque(3100);
    }
    else if (req == ENDURANCE && driveMode != ENDURANCE)
    {
        Serial.println("[ECU]   switching to ENDURANCE");
        sendParam();
        driveMode = ENDURANCE;
        throttle.setMaxTorque(1500);
    }
    else if (req == SKID_PAD && driveMode != SKID_PAD)
    {
        Serial.println("[ECU]   switching to SKID_PAD");
        sendParam();
        driveMode = SKID_PAD;
        throttle.setMaxTorque(1500);
    }
    else
    {
        Serial.println("[ECU]   no driveMode change");
    }
}

void ECU::updateInverter()
{
    // Cascadia Motion CM200DX Software docs:
    // https://www.cascadiamotion.com/uploads/5/1/3/0/51309945/0a-0163-02_sw_user_manual.pdf
    // Page 144-147 for msg id 0x0AA
    // Rate-limited serial prints
    static uint32_t lastUpdateInverterSerial = 0;
    const uint32_t now = millis();
    const uint32_t UPDATE_INVERTER_PRINT_INVERVAL = 250; // ms
    bool doPrint = (now - lastUpdateInverterSerial >= UPDATE_INVERTER_PRINT_INVERVAL);
    if (doPrint)
    {
        lastUpdateInverterSerial = now;
        Serial.print("[ECU] updateInverter raw buf: ");
        for (uint8_t i = 0; i < inMsg.len; i++) { Serial.print(inMsg.buf[i], HEX); Serial.print(' '); }
        Serial.println();
    }
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
    tractiveActive = (vsmState == WAIT || vsmState == READY || vsmState == MOTOR_RUNNING) && prechargeRelay && mainRelay && inverterLockout == UNLOCKED; // Determination if tractive is active and therefore if torque commands should be sent to the inverter
}

/** Sends the command to enable the inverter. A check for tractiveActive is assumed to have been done before calling */
void ECU::enableInverter()
{
    // Only enable the inverter when in VSM 4 and it has passed precharge
    Serial.println("[ECU] enableInverter");
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
    delay(5000);
}

void ECU::disableInverter()
{
    // Serial.println("[ECU] disableInverter");
    // outMsg.id = ControlCommandId;
    // outMsg.len = 8;
    // // Ignore torque
    // outMsg.buf[0] = 0;
    // outMsg.buf[1] = 0;
    // // Ignore speed
    // outMsg.buf[2] = 0;
    // outMsg.buf[3] = 0;
    // // Command forward
    // outMsg.buf[4] = FORWARD;
    // // Set enable bit to disable
    // outMsg.buf[5] = DISABLED;
    // // Ignore torque limit
    // outMsg.buf[6] = 0;
    // outMsg.buf[7] = 0;
    // motorCAN.write(outMsg);
}

void ECU::motorCommand(const int torque)
{
    // Rate-limited serial prints
    static uint32_t lastMotorCommandSerial = 0;
    const uint32_t now = millis();
    const uint32_t MOTOR_COMMAND_PRINT_INTERVAL = 250; // ms
    bool doPrint = (now - lastMotorCommandSerial >= MOTOR_COMMAND_PRINT_INTERVAL);
    if (doPrint)
    {
        Serial.print("[ECU] motorCommand torque="); Serial.println(torque);
        Serial.print("[ECU]   tractiveActive="); Serial.print(tractiveActive);
        Serial.print(" invEnabled="); Serial.print(inverterEnabled);
        Serial.print(" driveState="); Serial.println(driveState);
    }
    if (tractiveActive && !inverterEnabled)
    {
        if (doPrint) Serial.println("[ECU]   -> enableInverter");
        enableInverter();
    }
    if (driveState)
    {
        updateBTOverride(torque);
    }
    if (tractiveActive && driveState && !BTOverride) // 
    {
        if (doPrint) Serial.println("[ECU]   -> send torque cmd");
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
        // delay(2000);
    }
    else if (tractiveActive && !driveState)
    {
        // Stop the motor from spinning with a 0-torque command, which is identical to the "empty" enable command
        if (doPrint) Serial.println("[ECU]   -> drive off, still enableInverter for safe stop");
        enableInverter();
    }
}

void ECU::updateBTOverride(const int torque)
{
    // Commented out while brake is not pressurized
    BTOverride = false; // Remove when brake line is fixed


    // Serial.print("[ECU] updateBTOverride torque="); Serial.println(torque);
    // if (BTOverride && !brake.getBrakeActive() && torque <= BTO_OFF_THRESHOLD)
    // {
    //     Serial.println("[ECU]   clearing BTOverride");
    //     BTOverride = false;
    // }
    // if (torque >= BTO_ON_THRESHOLD && !BTOverride && brake.getBrakeActive())
    // {
    //     Serial.println("[ECU]   setting BTOverride");
    //     BTOverride = true;
    // }
}

/** Throw an error to the data CAN bus and shutdown */
void ECU::throwError(const uint8_t code)
{
    // Rate-limited serial prints
    static uint32_t lastThrowErrorSerial = 0;
    const uint32_t now = millis();
    const uint32_t THROW_ERROR_PRINT_INTERVAL = 250; // ms
    bool doPrint = (now - lastThrowErrorSerial >= THROW_ERROR_PRINT_INTERVAL);
    if (doPrint)
    {
        // Serial.print("[ECU] throwError code=");
        // Serial.println(code);
    }
    outMsg.id = FaultId;
    outMsg.len = 1;
    outMsg.buf[0] = code;
    dataCAN.write(outMsg);
    // shutdown(); // Shutdown to trigger a change to driveState
}
