Here is a diagram of our control flow:

```mermaid
flowchart TB
    PHONE["📱 Phone App"]
    MAIN["**main.cpp**\n───────────────\nsetup()\nloop()\nforceLogTask()"]
    PPG["**PPGModule**\n───────────────\nbegin()\nupdate()\nisSessionActive()\ngetCompressionLevel()\ngetCompressionType()\nlogForceData()"]

    FC["**ForceControl**\n───────────────\nbegin()\nreadForce()\nsetForceSetpoint()\ngetDesiredRevolutions()\nstartPeakTracking()\ncomputeNextAmplitude()"]

    MC["**MotionControl**\n───────────────\nbegin()\nexecutePattern1–5()\nrunSineCycle()\nupdateMotorState()"]

    BTS["**BTS7960**\n───────────────\nbegin()\nset()\ncoast()\nbrake()"]

    ENC["**EncoderModule**\n───────────────\nbegin()\ngetPosition()\nresetPosition()"]

    LIM["**LimitSensors**\n───────────────\nbegin()\nisTriggered()\nclearFlags()\ncheckPendingTriggers()"]

    PID["**PIDController**\n───────────────\ninit()\ncompute()\nsetSetpoint()\nreset()"]

    CFG["**config.h**\n───────────────\nPin definitions\nPID gains\nTiming constants\nBLE UUIDs"]
    subgraph HW["Hardware"]
        MOTORS["DC Motors × 3"]
        ENCODERS["Encoders × 3"]
        FORCE_S["Force Sensors × 3"]
        LIMIT_S["Limit Switches × 3"]
        MAX30["PPG"]
        SD["SD Card"]
    end
    PHONE <-->|"BLE"| PPG
    PPG <-->|"Session state\nLevel / Type\nStop flag"| MAIN
    MAIN -->|"setForceSetpoint()"| FC
    FC -->|"getDesiredRevolutions()\namplitudes[]"| MAIN
    MAIN -->|"executePatternN()\nmotor[], amplitudes[]"| MC
    MAIN -->|"readForce()"| FC
    MAIN -->|"logForceData()"| PPG
    MC <-->|"compute()\nsetSetpoint()"| PID
    MC -->|"set() / coast()"| BTS
    MC -->|"getPosition()"| ENC
    MC -->|"isTriggered()"| LIM
    MC -->|"startPeakTracking()"| FC
    FC <-->|"compute()\nsetSetpoint()"| PID
    BTS --> MOTORS
    MOTORS --> ENCODERS
    ENCODERS --> ENC
    LIMIT_S --> LIM
    FORCE_S --> FC
    MAX30 --> PPG
    PPG --> SD