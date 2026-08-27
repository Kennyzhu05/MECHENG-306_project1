    #include <Arduino.h>
    #include "Motion.h"
    #include "Motor.h"
    #include "Encoder.h"

    // ---- Tunable PI gains and limits ----
    // float KP = 0.9; // previous 0.7
    // float KI = 0.05;
    float KP_A = 0.43;
    float KP_B = 0.43;
    float KI_A = 0.38;
    float KI_B = 0.38;
    float SYNC_KP = 3;
    long SYNC_START_COUNTS = 150;
    int MIN_PWM = 100;      // below this, motors may not overcome static friction
    int MAX_PWM = 240;
    long POSITION_TOLERANCE = 12;   // encoder counts considered "close enough"
    int SETTLE_SAMPLES = 5;        // consecutive in-tolerance loops before stopping
    unsigned long MOVE_TIMEOUT_MS = 100000;   // safety cutoff
    const long MAX_SAFE_TARGET = 30000; // Variable used to prevent integer overflow

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

        if (speedPercent == 0)
        {
            return 0;
        }

        int pwm =
            (int)round((speedPercent / 100.0f) * MAX_PWM);

        // Any non-zero movement needs enough power
        // to overcome static friction.
        if (pwm < MIN_PWM)
        {
            pwm = MIN_PWM;
        }

        return pwm;
    }

    // Converts a raw PI output into a signed PWM command: capped at
    // MAX_PWM, and bumped up to MIN_PWM if it's small-but-nonzero so the
    // motor doesn't stall in the "should be moving but isn't" zone.
    long ACCEL_ZONE_COUNTS = 1000;
    long DECEL_ZONE_COUNTS = 1000; // distance-to-target (counts) where speed starts ramping down

    static int toMotorCommand(float piOutput, long error, long masterTravelled, long masterTarget, float motorRatio)
    {
        long masterRemaining = masterTarget - masterTravelled;

        if (masterRemaining < 0)
        {
            masterRemaining = 0;
        }

        int masterSpeedCap = moveMaxPWM;


        // ==============================
        // Acceleration ramp
        // ==============================
        if (masterTravelled < ACCEL_ZONE_COUNTS)
        {
            float ramped = MIN_PWM + (float)(moveMaxPWM - MIN_PWM) * ((float)masterTravelled /
                (float)ACCEL_ZONE_COUNTS);

            masterSpeedCap = (int)round(ramped);
        }


        // ==============================
        // Deceleration ramp
        // ==============================
        if (masterRemaining < DECEL_ZONE_COUNTS)
        {
            float ramped = MIN_PWM + (float)(moveMaxPWM - MIN_PWM) * ((float)masterRemaining /
                (float)DECEL_ZONE_COUNTS);

            int decelCap = (int)round(ramped);

            if (decelCap < masterSpeedCap)
            {
                masterSpeedCap = decelCap;
            }
        }


        // =====================================================
        // Scale speed according to this motor's required travel
        // =====================================================
        int speedCapNow = (int)round((float)masterSpeedCap * motorRatio);

        if (speedCapNow > moveMaxPWM)
        {
            speedCapNow = moveMaxPWM;
        }


        // ==============================
        // Apply speed limit
        // ==============================
        int pwm = (int)round(piOutput);

        pwm = clampInt(pwm, -speedCapNow, speedCapNow);


        // ==============================
        // Overcome static friction
        // ==============================
        if (pwm > 0 && pwm < MIN_PWM)
        {
            pwm = (MIN_PWM > speedCapNow) ? speedCapNow : MIN_PWM;
        }
        else if (pwm < 0 && pwm > -MIN_PWM)
        {
            pwm = (MIN_PWM > speedCapNow) ? -speedCapNow : -MIN_PWM;
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

        // ======================================================
        // SHARED MASTER TRAJECTORY
        // ======================================================
        long absTargetA = labs(targetA);
        long absTargetB = labs(targetB);

        // Longest motor movement is the master trajectory
        long masterTarget = (absTargetA >= absTargetB) ? absTargetA : absTargetB;


        // ======================================================
        // MASTER PROGRESS
        // ======================================================

        long masterTravelled = 0;

        if (absTargetA >= absTargetB)
        {
            masterTravelled = labs(encoderA);
        }
        else
        {
            masterTravelled = labs(encoderB);
        }

        // Prevent progress going outside the trajectory
        if (masterTravelled > masterTarget)
        {
            masterTravelled = masterTarget;
        }


        // ======================================================
        // MOTOR TRAVEL RATIOS
        // ======================================================

        float ratioA = 0.0f;
        float ratioB = 0.0f;

        if (masterTarget > 0)
        {
            ratioA = (float)absTargetA / (float)masterTarget;

            ratioB = (float)absTargetB / (float)masterTarget;
        }


        // ======================================================
        // PI OUTPUT -> PWM
        // ======================================================

        int pwmA = toMotorCommand(outputA, errorA, masterTravelled, masterTarget, ratioA);
        int pwmB = toMotorCommand(outputB, errorB, masterTravelled, masterTarget, ratioB);


        // ======================================================
        // STOP INDIVIDUAL MOTOR WHEN IT REACHES TARGET
        // ======================================================

        if (labs(errorA) <= POSITION_TOLERANCE)
        {
            pwmA = 0;
        }

        if (labs(errorB) <= POSITION_TOLERANCE)
        {
            pwmB = 0;
        }


        // ======================================================
        // PROGRESS SYNCHRONISATION
        //
        // Compare percentage progress.
        // If one motor gets ahead, slow that motor slightly.
        // Do NOT speed the other motor above its trajectory cap.
        // ======================================================
        float progressError = 0.0f;
        bool syncActive = false;

        if (masterTravelled > SYNC_START_COUNTS && absTargetA > POSITION_TOLERANCE &&
            absTargetB > POSITION_TOLERANCE && labs(errorA) > POSITION_TOLERANCE &&
            labs(errorB) > POSITION_TOLERANCE)
        {
            float progressA = (float)encoderA / (float)targetA;

            float progressB = (float)encoderB / (float)targetB;

            progressError = progressA - progressB;
            syncActive = true;


            int syncCorrection = (int)round( SYNC_KP * fabs(progressError) * moveMaxPWM);

            // Don't let sync correction become excessive
            syncCorrection = clampInt( syncCorrection, 0, moveMaxPWM / 4);


            // A is ahead -> slow A
            if (progressError > 0.0f)
            {
                int magnitude = abs(pwmA);

                magnitude -= syncCorrection;

                if (magnitude < 0)
                {
                    magnitude = 0;
                }

                pwmA = (errorA >= 0) ? magnitude : -magnitude;
            }

            // B is ahead -> slow B
            else if (progressError < 0.0f)
            {
                int magnitude = abs(pwmB);

                magnitude -= syncCorrection;

                if (magnitude < 0)
                {
                    magnitude = 0;
                }

                pwmB = (errorB >= 0) ? magnitude : -magnitude;
            }
        }

        // ======================================================
        // STATIC FRICTION / LOW PWM HANDLING
        // ======================================================

        // Is each motor currently ahead of the desired trajectory?
        bool motorAAhead = syncActive && progressError > 0.0f;
        bool motorBAhead =  syncActive && progressError < 0.0f;


        // ------------------------------
        // Motor A
        // ------------------------------

        if (labs(errorA) <= POSITION_TOLERANCE)
        {
            pwmA = 0;
        }
        else if (pwmA != 0 && abs(pwmA) < MIN_PWM)
        {
            if (motorAAhead)
            {
                // A is ahead, so pause it
                pwmA = 0;
            }
            else
            {
                // A still needs to catch up.
                // Give it enough power to actually move.
                pwmA = (errorA > 0) ? MIN_PWM : -MIN_PWM;
            }
        }


        // ------------------------------
        // Motor B
        // ------------------------------

        if (labs(errorB) <= POSITION_TOLERANCE)
        {
            pwmB = 0;
        }
        else if (pwmB != 0 && abs(pwmB) < MIN_PWM)
        {
            if (motorBAhead)
            {
                // B is ahead, so pause it
                pwmB = 0;
            }
            else
            {
                // B still needs to catch up
                pwmB = (errorB > 0) ? MIN_PWM : -MIN_PWM;
            }
        }

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