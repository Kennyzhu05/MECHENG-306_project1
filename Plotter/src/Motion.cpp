    #include <Arduino.h>
    #include "Motion.h"
    #include "Motor.h"
    #include "Encoder.h"

    // ---- Tunable PI gains and limits ----
    // float KP = 0.9; // previous 0.7
    // float KI = 0.05;
    float KP_A = 0.6;
    float KP_B = 0.6;
    float KI_A = 0.4;
    float KI_B = 0.4;
    float SYNC_KP = 0.5;
    int MIN_PWM = 80;      // below this, motors may not overcome static friction
    int MAX_PWM = 240;
    long POSITION_TOLERANCE = 5;   // encoder counts considered "close enough"
    int SETTLE_SAMPLES = 5;        // consecutive in-tolerance loops before stopping
    unsigned long MOVE_TIMEOUT_MS = 100000;   // safety cutoff
    const long MAX_SAFE_TARGET = 30000; //Variable used to prevent integer overflow

    // ---- Encoder factors ----
    const float X_COUNTS_PER_MM_EXISTING = 20041.8 / 220.0;
    const float Y_COUNTS_PER_MM_EXISTING = 12339.2 / 130.0;

    const float A_X_COUNTS_PER_MM = 20276.4 / 220.0;
    const float B_X_COUNTS_PER_MM = 19807.8 / 220.0;
    const float A_Y_COUNTS_PER_MM = 12393.6 / 130.0;
    const float B_Y_COUNTS_PER_MM = 12285.6 / 130.0;
    

    // ---- Persistent motion state (replaces the old while-loop locals) ----
    static long targetA = 0;
    static long targetB = 0;
    static float integralA = 0;
    static float integralB = 0;
    static unsigned long moveStartTime = 0;
    static unsigned long lastTickTime = 0;
    static int settledCount = 0;
    static bool motionActive = false;
    static MotionResult lastResult = MotionResult::NONE;

    static int clampInt(int value, int lo, int hi)
    {
        if (value < lo) return lo;
        if (value > hi) return hi;
        return value;
    }

    // ---- Per-move speed cap (replaces flat MAX_PWM as the ceiling used
    // inside toMotorCommand for a given move; MAX_PWM remains the
    // absolute hardware ceiling that speedPercent=100 maps to) ----
    static int moveMaxPWM = MAX_PWM;

    static int speedPercentToPWM(int speedPercent)
    {
        speedPercent = clampInt(speedPercent, 0, 100);
        return (int)round((speedPercent / 100.0) * MAX_PWM);
    }

    // Converts a raw PI output into a signed PWM command: capped at
    // MAX_PWM, and bumped up to MIN_PWM if it's small-but-nonzero so the
    // motor doesn't stall in the "should be moving but isn't" zone.
    long ACCEL_ZONE_COUNTS = 800;
    long DECEL_ZONE_COUNTS = 800; // distance-to-target (counts) where speed starts ramping down

    static int toMotorCommand(float piOutput, long error, long target)
    {
        long absError = labs(error);
        long absTarget = labs(target);

        // Distance travelled since beginning of move
        long travelled = absTarget - absError;

        if (travelled < 0)
        {
            travelled = 0;
        }

        int speedCapNow = moveMaxPWM;


        // ==============================
        // Acceleration ramp
        // ==============================
        if (travelled < ACCEL_ZONE_COUNTS)
        {
            float ramped = MIN_PWM + (float)(moveMaxPWM - MIN_PWM) * 
                ((float)travelled / (float)ACCEL_ZONE_COUNTS);

            speedCapNow = (int)round(ramped);
        }


        // ==============================
        // Deceleration ramp
        // ==============================
        if (absError < DECEL_ZONE_COUNTS)
        {
            float ramped = MIN_PWM + (float)(moveMaxPWM - MIN_PWM) *
                ((float)absError / (float)DECEL_ZONE_COUNTS);

            int decelCap = (int)round(ramped);

            // Use whichever limit is smaller
            if (decelCap < speedCapNow)
            {
                speedCapNow = decelCap;
            }
        }


        // ==============================
        // Apply speed limit
        // ==============================
        int pwm = (int)piOutput;

        pwm = clampInt(pwm, -speedCapNow, speedCapNow);


        // Overcome static friction
        if (pwm > 0 && pwm < MIN_PWM)
        {
            pwm = MIN_PWM > speedCapNow
                ? speedCapNow
                : MIN_PWM;
        }
        else if (pwm < 0 && pwm > -MIN_PWM)
        {
            pwm = MIN_PWM > speedCapNow
                ? -speedCapNow
                : -MIN_PWM;
        }

        return pwm;
    }

    void startMotion(long targetXCounts, long targetYCounts, int speedPercent)
    {
        moveMaxPWM = speedPercentToPWM(speedPercent);

        targetA = 0;
        targetB = 0;
        float targetXmm = (float)targetXCounts / X_COUNTS_PER_MM_EXISTING;
        float targetYmm = (float)targetYCounts / Y_COUNTS_PER_MM_EXISTING;
        
        // Calculate targets first
        long calculatedTargetA = (long)round(targetXmm * A_X_COUNTS_PER_MM + targetYmm * A_Y_COUNTS_PER_MM);
        long calculatedTargetB = (long)round(targetXmm * B_X_COUNTS_PER_MM - targetYmm * B_Y_COUNTS_PER_MM);

        // Check that the requested move is safe
        if (abs(calculatedTargetA) > MAX_SAFE_TARGET ||
            abs(calculatedTargetB) > MAX_SAFE_TARGET)
        {
            Serial.println("ERROR: Requested movement will cause integer overflow");

            motionActive = false;
            lastResult = MotionResult::INVALID_TARGET;

            stopMotors();
            return;
        }

        // // Only drive motors required
        // if ((targetXCounts > 0 && targetYCounts > 0) || (targetXCounts < 0 && targetYCounts < 0))
        // {
        //     // Only Motor A contributes
        //     targetA = (long)round(targetXmm * A_X_COUNTS_PER_MM + targetYmm * A_Y_COUNTS_PER_MM);
        //     targetB = 0;
        // } else if ((targetXCounts > 0 && targetYCounts < 0) || (targetXCounts < 0 && targetYCounts > 0))
        // {
        //     // Only Motor B contributes
        //     targetA = 0;
        //     targetB = (long)round(targetXmm * B_X_COUNTS_PER_MM - targetYmm * B_Y_COUNTS_PER_MM);
        // } else
        // {
        //     targetA = (long)round(targetXmm * A_X_COUNTS_PER_MM + targetYmm * A_Y_COUNTS_PER_MM);
        //     targetB = (long)round(targetXmm * B_X_COUNTS_PER_MM - targetYmm * B_Y_COUNTS_PER_MM);
        // }
        
        targetA = (long)round(targetXmm * A_X_COUNTS_PER_MM + targetYmm * A_Y_COUNTS_PER_MM);
        targetB = (long)round(targetXmm * B_X_COUNTS_PER_MM - targetYmm * B_Y_COUNTS_PER_MM);

        resetEncoders();
        integralA = 0;
        integralB = 0;
        settledCount = 0;
        moveStartTime = millis();
        lastTickTime = moveStartTime;
        lastResult = MotionResult::NONE;
        motionActive = true;

        Serial.println("time_ms,encoderA,targetA,errorA,pwmA,encoderB,targetB,errorB,pwmB");
    }

    void abortMotion()
    {
        motionActive = false;
        stopMotors();
    }

    bool isMotionActive()
    {
        return motionActive;
    }

    bool isMotionTimedOut()
    {
        if (!motionActive)
        {
            return false;
        }

        unsigned long now = millis();
        return (now - moveStartTime >= MOVE_TIMEOUT_MS);
    }

    bool isMotionDone()
    {
        return !motionActive;
    }

    MotionResult getLastMotionResult()
    {
        return lastResult;
    }

    // Runs exactly one PI iteration. This is the old while-loop body,
    // unchanged in its control logic -- only the looping mechanism moved
    // out to the caller (loop() calls this repeatedly instead of a
    // blocking while(true) here).
    void updateMotion()
    {
        if (!motionActive)
        {
            return;
        }

        updateEncoders();

        unsigned long now = millis();
        float dt = (now - lastTickTime) / 1000.0;
        if (dt <= 0) dt = 0.001;   // guard against a zero dt on fast loops
        lastTickTime = now;

        long encoderA = getLeftEncoderCount();
        long encoderB = getRightEncoderCount();

        long errorA = targetA - encoderA;
        long errorB = targetB - encoderB;


        // Both motors settled -> done
        if (abs(errorA) <= POSITION_TOLERANCE && abs(errorB) <= POSITION_TOLERANCE)
        {
            stopMotors();

            settledCount++;
            if (settledCount >= SETTLE_SAMPLES)
            {
                stopMotors();
                motionActive = false;
                lastResult = MotionResult::SETTLED;
            }

            return;
        }
        else
        {
            settledCount = 0;
        }


        // --- Motor A ---
        float outputA = KP_A * errorA + KI_A * integralA;
        
        bool satPosA = (outputA > moveMaxPWM && errorA > 0);
        bool satNegA = (outputA < -moveMaxPWM && errorA < 0);
        if (!satPosA && !satNegA)
        {
            integralA += errorA * dt;
        }

        // --- Motor B ---
        float outputB = KP_B * errorB + KI_B * integralB;
        
        bool satPosB = (outputB > moveMaxPWM && errorB > 0);
        bool satNegB = (outputB < -moveMaxPWM && errorB < 0);
        if (!satPosB && !satNegB)
        {
            integralB += errorB * dt;
        }

        // Once a motor is within tolerance, stop driving it so it
        // doesn't hunt back and forth while the other axis finishes
        // if (abs(errorA) <= POSITION_TOLERANCE) pwmA = 0;
        // if (abs(errorB) <= POSITION_TOLERANCE) pwmB = 0;
        int pwmA = toMotorCommand(outputA, errorA, targetA);
        int pwmB = toMotorCommand(outputB, errorB, targetB);

        // Debugging monitor messages
        static unsigned long lastPrintTime = 0;
        const unsigned long PRINT_INTERVAL_MS = 100;

        if (now - lastPrintTime >= PRINT_INTERVAL_MS)
        {
            Serial.print(now - moveStartTime);
            Serial.print(",");

            Serial.print(encoderA);
            Serial.print(",");
            Serial.print(targetA);
            Serial.print(",");
            Serial.print(errorA);
            Serial.print(",");
            Serial.print(pwmA);
            Serial.print(",");

            Serial.print(encoderB);
            Serial.print(",");
            Serial.print(targetB);
            Serial.print(",");
            Serial.print(errorB);
            Serial.print(",");
            Serial.println(pwmB);

            lastPrintTime = now;
        }

        driveMotorA(pwmA);
        driveMotorB(pwmB);

        
        if (now - moveStartTime >= MOVE_TIMEOUT_MS)
        {
            // Safety timeout: stall, disconnected encoder, sign inversion, etc.
            stopMotors();
            motionActive = false;
            lastResult = MotionResult::TIMED_OUT;
            return;
        }
    }

    // ---- Blocking wrapper kept for bench testing outside the FSM ----
    void moveXY(long targetXCounts, long targetYCounts, int speedPercent)
    {
        startMotion(targetXCounts, targetYCounts, speedPercent);
        while (isMotionActive())
        {
            updateMotion();
        }
    }

    void moveX(long targetCounts, int speedPercent)
    {
        moveXY(targetCounts, 0, speedPercent);
    }

    void moveY(long targetCounts, int speedPercent)
    {
        moveXY(0, targetCounts, speedPercent);
    }