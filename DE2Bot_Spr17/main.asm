; This code uses the timer interrupt for the control code.
ORG 0                  ; Jump table is located in mem 0-4
    JUMP   Init        ; Reset vector
    RETI               ; Sonar interrupt (unused)
    JUMP   CTimer_ISR  ; Timer interrupt
    RETI               ; UART interrupt (unused)
    RETI               ; Motor stall interrupt (unused)

;***************************************************************
;* Initialization
;***************************************************************
Init:
    ; Always a good idea to make sure the robot
    ; stops in the event of a reset.
    LOAD   Zero
    OUT    LVELCMD     ; Stop motors
    OUT    RVELCMD
    OUT    SONAREN     ; Disable sonar (optional)
    OUT    BEEP        ; Stop any beeping (optional)

    CALL   SetupI2C    ; Configure the I2C to read the battery voltage
    CALL   BattCheck   ; Get battery voltage (and end if too low).
    OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
    ; This loop will wait for the user to toggle SW17.  Note that
    ; SCOMP does not have direct access to SW17; it only has access
    ; to the SAFETY signal contained in XIO.
    IN     XIO         ; XIO contains SAFETY signal
    AND    Mask4       ; SAFETY signal is bit 4
    JPOS   WaitForUser ; If ready, jump to wait for PB3
    IN     TIMER       ; We'll use the timer value to
    AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
    SHIFT  8           ; Shift over to LED17
    OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
    JUMP   WaitForSafety

WaitForUser:
     ; This loop will wait for the user to press PB3, to ensure that
     ; they have a chance to prepare for any movement in the main code.
    IN     TIMER        ; We'll blink the LEDs above PB3
    AND    Mask1
    SHIFT  5            ; Both LEDG6 and LEDG7
    STORE  Temp         ; (overkill, but looks nice)
    SHIFT  1
    OR     Temp
    OUT    XLEDS
    IN     XIO          ; XIO contains KEYs
    AND    Mask2        ; KEY3 mask (KEY0 is reset and can't be read)
    JPOS   WaitForUser  ; not ready (KEYs are active-low, hence JPOS)
    LOAD   Zero
    OUT    XLEDS        ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main:
    OUT     RESETPOS            ; reset odometer in case wheels moved after programming
    LOAD    Zero
    STORE   DVel                ; desired forward velocity
    IN      THETA
    STORE   DTheta              ; desired heading
    LOADI   10                  ; 10ms * 10 = 0.1s rate, or 10Hz.
    OUT     CTIMER              ; turn on timer peripheral
    LOAD    Play                ; Set forward button
    STORE   BtnForward          ; ""
    LOAD    FastF               ; Set turn right button
    STORE   BtnTurnRight        ; ""
    LOAD    Rew                 ; Set turn left button
    STORE   BtnTurnLeft         ; ""
    LOAD    VolUp               ; Set forward small button
    STORE   BtnForwardSmall     ; ""
    LOAD    BtnStop             ; Set turn right small button
    STORE   BtnTurnRightSmall   ; ""
    LOAD    BtnPause            ; Set turn left small button
    STORE   BtnTurnLeftSmall    ; ""
    LOAD    PrevChan			; Set parallel park button
    STORE   BtnParkParallel		; ""
    LOAD    Mute				; Set parallel park button
    STORE   BtnParkPerp			; ""
    LOAD    ChanUp				; Set move to perp button
    STORE   BtnForwardPerp		; ""
    LOAD    BtnZero			; Set move to perp button
    STORE   BtnParkPerpFirst	; ""

RemoteLoop:
    IN      IR_LO                   ; Capture remote button press
    STORE   Remote                  ; ""
    OUT     LCD
    OUT     IR_LO                   ; Clear remote code
    LOADI   -1                      ; Reset perpindicular space
    STORE   PerpSpace               ; ""
    LOAD    Zero                    ; Clear Auto params
    STORE   AutoDistance            ; ""
    STORE   AutoAngle               ; ""
    CALL    WaitForForwardBtn       ; Check forward movement
    CALL    WaitForForwardSmallBtn  ; Check forward small movement
    CALL    WaitForRightTurnBtn     ; Check right turn movement
    CALL    WaitForRightTurnSmallBtn; Check right turn small movement
    CALL    WaitForLeftTurnBtn      ; Check left turn movement
    CALL    WaitForLeftTurnSmallBtn ; Check left turn small movement
    CALL    WaitForParallelBtn		; Check for parallel park button
    CALL    WaitForPerpBtn			; Check for perpindicular button
    CALL    WaitForForwardPerpBtn   ; Check for perpindicular button
    CALL    WaitForFirstPerpBtn		; Check for perpindicular button
    CALL    CheckPerpAuto           ; Get perpindicular space if button pressed
    LOAD    PerpSpace               ; ""
    JNEG    RemoteLoop              ; If no space selected repeat
AutoPerpAc:
    CALL    GetToFirstPerp  ; Do actions to get to the first perpindicular space
    LOAD    AutoAngle       ; Load the angle correction
	OUT     LCD
	LOAD    Zero
	STORE   Angle
	LOAD    AutoAngle       ; Load the angle correction
	JPOS    CorrectLeft
	JNEG    CorrectRight
CorrectLeft:
	LOAD    AutoAngle
	ADDI    -5
	STORE   AutoAngle
	LOAD    Angle
	ADDI    -1
	STORE   Angle
	LOAD    AutoAngle
	JPOS    CorrectLeft
	JUMP    Finish
CorrectRight:  
	LOAD    AutoAngle   
	ADDI    5
	STORE   AutoAngle
	LOAD    Angle
	ADDI    1
	STORE   Angle
	LOAD    AutoAngle
	JNEG    CorrectRight
Finish:
	LOAD    Angle
	OUT     LCD
	LOADI   40
	STORE   WaitTime
	CALL    WaitFT
	LOAD    Angle
	CALL    ABS
	ADDI    -7
	JPOS    DontTurn
    CALL    Turn            ; Angle correction
DontTurn:
    LOAD    DistPerpFour    ; Load distane to the first perpindicular parking spot
    STORE   Distance        ; Store it
    CALL    Traverse        ; Go to it
    LOAD    AutoDistance    ; Load the distance to the perpindicular spot selected
    STORE   Distance        ; Store that shit
    CALL    Traverse        ; Go to it already
    CALL    ParkPerp        ; Park in that bitch
    JUMP    RemoteLoop      ; Loopty loop



; Timer ISR.  Currently just calls the movement control code.
; You could, however, do additional tasks here if desired.
CTimer_ISR:
    CALL   ControlMovement
    RETI   ; return from ISR


; Control code.  If called repeatedly, this code will attempt
; to control the robot to face the angle specified in DTheta
; and match the speed specified in DVel
DTheta:    DW 0
DVel:      DW 0
ControlMovement:
    LOADI  50               ; used later to get a +/- constant
    STORE  MaxVal
    CALL   GetThetaErr ; get the heading error
    ; A simple way to get a decent velocity value
    ; for turning is to multiply the angular error by 4
    ; and add ~50.
    SHIFT  2
    STORE  CMAErr           ; hold temporarily
    SHIFT  2                ; multiply by another 4
    CALL   CapValue         ; get a +/- max of 50
    ADD    CMAErr
    STORE  CMAErr


    ; For this basic control method, simply take the
    ; desired forward velocity and add a differential
    ; velocity for each wheel when turning is needed.
    LOADI  510
    STORE  MaxVal
    LOAD   DVel
    CALL   CapValue         ; ensure velocity is valid
    STORE  DVel             ; overwrite any invalid input
    ADD    CMAErr
    CALL   CapValue         ; ensure velocity is valid
    OUT    RVELCMD
    LOAD   CMAErr
    CALL   Neg              ; left wheel gets negative differential
    ADD    DVel
    CALL   CapValue
    OUT    LVELCMD

    RETURN
    CMAErr: DW 0            ; holds angle error velocity

; Returns the current angular error wrapped to +/-180
GetThetaErr:
    ; convenient way to get angle error in +/-180 range is
    ; ((error + 180) % 360 ) - 180
    IN     THETA
    SUB    DTheta           ; actual - desired angle
    CALL   Neg              ; desired - actual angle
    ADDI   180
    CALL   Mod360
    ADDI   -180
    RETURN

; caps a value to +/-MaxVal
CapValue:
    SUB     MaxVal
    JPOS    CapVelHigh
    ADD     MaxVal
    ADD     MaxVal
    JNEG    CapVelLow
    SUB     MaxVal
    RETURN
CapVelHigh:
    LOAD    MaxVal
    RETURN
CapVelLow:
    LOAD    MaxVal
    CALL    Neg
    RETURN
    MaxVal: DW 510

;***************************************************************
;* Subroutines
;***************************************************************

;*******************************************************************************
; GetWheelEncoderDist: adds the wheel encoder distances together and result is
; in the acc.
;*******************************************************************************
GetWheelEncoderDist:
    IN      LPOS    ; Get LPOS
    STORE   Temp    ; Store temporarily
    IN      RPOS    ; Get RPOS
    CALL    ABS     ; Absolute that shit
    ADD     Temp    ; Add both together
    RETURN          ; Bail out

GetToFirstPerp:
    LOAD    DistPerpOne     ; Load distance to the first intersection
    STORE   Distance        ; Store it
    CALL    Traverse        ; Go to it
    LOADI   -86             ; Turn almost 90
    STORE   Angle           ; ""
    CALL    Turn            ; ""
    LOAD    DistPerpTwo     ; Load distane to the second intersection
    STORE   Distance        ; Store it
    CALL    Traverse        ; Go to it
    LOADI   86              ; Turn almost 90
    STORE   Angle           ; ""
    LOAD 	mask5			;G
	OUT 	SONAREN
    CALL    Turn            ; ""
    CALL 	SonarCheck		;
    LOAD	sonarAVG
    STORE	sonarValueGood
    OUT     SSEG1
    LOAD    Zero
    STORE   sonarAVG
    LOAD    DistPerpThree   ; Load distane to the first perpindicular parking spot
    STORE   Distance        ; Store it
    CALL    Traverse        ; Go to it
    CALL	SonarCheck
    LOAD    sonarAVG
    OUT     SSEG2
    SUB 	sonarValueGood
    STORE 	AutoAngle
    LOAD    Zero
    STORE   sonarAVG
	OUT 	SONAREN
    RETURN                  ; Bail out
sonarValueGood: 	DW 0

CheckPerpAuto:
    LOAD    Remote              ; Load Remote and check buttons 1-7
    SUB     BtnOne              ; ""
    JZERO   SetOne              ; ""
    LOAD    Remote              ; ""
    SUB     BtnTwo              ; ""
    JZERO   SetTwo              ; ""
    LOAD    Remote              ; ""
    SUB     BtnThree            ; ""
    JZERO   SetThree            ; ""
    LOAD    Remote              ; ""
    SUB     BtnFour             ; ""
    JZERO   SetFour             ; ""
    LOAD    Remote              ; ""
    SUB     BtnFive             ; ""
    JZERO   SetFive             ; ""
    LOAD    Remote              ; ""
    SUB     BtnSix              ; ""
    JZERO   SetSix              ; ""
    LOAD    Remote              ; ""
    SUB     BtnSeven            ; ""
    JZERO   SetSeven            ; ""
    JUMP    EndCheckPerpAuto    ; ""
SetOne:
    LOADI   6                   ; Space to go to
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    LOAD    DistToSeven         ; Distance to travel
    STORE   AutoDistance        ; ""
;     LOADI   6                   ; Angle correction
;     STORE   AutoAngle           ; ""
    JUMP    EndCheckPerpAuto    ; Bail out
SetTwo:
    LOADI   5                   ; Space to go to
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    LOAD    DistToSix           ; Distance to travel
    STORE   AutoDistance        ; ""
;     LOADI   5                   ; Angle correction
;     STORE   AutoAngle           ; ""
    JUMP    EndCheckPerpAuto    ; Bail out
SetThree:
    LOADI   4                   ; Space to go to
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    LOAD    DistToFive          ; Distance to travel
    STORE   AutoDistance        ; ""
;     LOADI   5                   ; Angle correction
;     STORE   AutoAngle           ; ""
    JUMP    EndCheckPerpAuto    ; Bail out
SetFour:
    LOADI   3                   ; Space to go to
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    LOAD    DistToFour          ; Distance to travel
    STORE   AutoDistance        ; ""
;     LOADI   5                   ; Angle correction
;     STORE   AutoAngle           ; ""
    JUMP    EndCheckPerpAuto    ; Bail out
SetFive:
    LOADI   2                   ; Space to go to
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    LOAD    DistToThree         ; Distance to travel
    STORE   AutoDistance        ; ""
;     LOADI   5                   ; Angle correction
;     STORE   AutoAngle           ; ""
    JUMP    EndCheckPerpAuto    ; Bail out
SetSix:
    LOADI   1                   ; Space to go to
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    LOAD    DistToTwo           ; Distance to travel
    STORE   AutoDistance        ; ""
    JUMP    EndCheckPerpAuto    ; Bail out
SetSeven:
    LOADI   0                   ; Space to go to
    STORE   PerpSpace           ; ""
    STORE   PerpSpace           ; ""
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
    STORE   AutoDistance        ; Distance to travel
EndCheckPerpAuto:
    RETURN                      ; Bail out

ParkPerp:
    LOADI   -86         ; Turn almost 90
    STORE   Angle       ; ""
    CALL    Turn        ; ""
    LOADI   610         ; Distance to pull into parking spot
    STORE   Distance    ; Store it
    CALL    Traverse    ; Park baby
    RETURN              ; Bail out

;*******************************************************************************
; SonarCheck: check the right facing sonar (S5)
;*******************************************************************************
SonarCheck:
	LOADI   1
	STORE   WaitTime
	CALL    WaitFT
    IN 		DIST5
    STORE 	sonarAVG
    OUT     LCD
    RETURN
    
sonarAVG:		DW 0;
;*******************************************************************************
; WaitForPerpBtn: parks parallel
;*******************************************************************************
WaitForPerpBtn:
    LOAD    Remote          		; Get remote code
    SUB     BtnParkPerp      	    ; Subtract code for button parallel park
    JNEG    EndWaitPerp  			; If negative or positive end
    JPOS    EndWaitPerp  			; ""
    CALL    ParkPerp      			; Parallel Park
    LOAD    Zero            		; Clear Remote code
    STORE   Remote          		; ""
EndWaitPerp:
    RETURN                  		; Bail out

;*******************************************************************************
; WaitForParallelBtn: parks parallel
;*******************************************************************************
WaitForParallelBtn:
    LOAD    Remote          		; Get remote code
    SUB     BtnParkParallel      	; Subtract code for button parallel park
    JNEG    EndWaitParallel  		; If negative or positive end
    JPOS    EndWaitParallel  		; ""
    CALL    ParkParallel			; Parallel Park
    LOAD    Zero            		; Clear Remote code
    STORE   Remote          		; ""
EndWaitParallel:
    RETURN                  		; Bail out
    
ParkParallel:
    LOADI   -85         ; Turn almost 90
    STORE   Angle       ; ""
    CALL    Turn        ; ""
    LOADI   380         ; Distance to pull into parking spot
    STORE   Distance    ; Store it
    CALL    Traverse    ; Park baby
   	LOADI   85         	; Turn almost 90
    STORE   Angle       ; ""
    CALL    Turn		; ""
    RETURN              ; Bail out

;*******************************************************************************
; WaitForFirstPerpBtn: moves forward two feet when zero is pressed
;*******************************************************************************
WaitForFirstPerpBtn:
    LOAD    Remote          				; Get remote code
    SUB     BtnParkPerpFirst      			; Subtract code for button forward
    JNEG    EndWaitForFirstPerpBtn  		; If negative or positive end
    JPOS    EndWaitForFirstPerpBtn  		; ""
    CALL    GetToFirstPerp        			; Start moving
    LOAD    DistPerpFour    ; Load distane to the first perpindicular parking spot
    STORE   Distance        ; Store it
    CALL    Traverse        ; Go to it
    LOAD    Zero            				; Clear Remote code
    STORE   Remote          				; ""
EndWaitForFirstPerpBtn:
    RETURN                  				; Bail out
    
;*******************************************************************************
; WaitForForwardPerpBtn: moves forward two feet when zero is pressed
;*******************************************************************************
WaitForForwardPerpBtn:
    LOAD    Remote          				; Get remote code
    SUB     BtnForwardPerp      			; Subtract code for button forward
    JNEG    EndWaitForForwardPerpBtn  		; If negative or positive end
    JPOS    EndWaitForForwardPerpBtn  		; ""
    LOAD    DistBetweenPerp     			; Distance between spots
    STORE   Distance        				; ""
    CALL    Traverse        				; Start moving
    LOAD    Zero            				; Clear Remote code
    STORE   Remote          				; ""
EndWaitForForwardPerpBtn:
    RETURN                  				; Bail out
    
    
;*******************************************************************************
; WaitForForwardBtn: moves forward two feet when zero is pressed
;*******************************************************************************
WaitForForwardBtn:
    LOAD    Remote          		; Get remote code
    SUB     BtnForward      		; Subtract code for button forward
    JNEG    EndWaitForward  		; If negative or positive end
    JPOS    EndWaitForward  		; ""
    LOAD    DistStartToParallel     ; Go to first perp
    STORE   Distance        		; ""
    CALL    Traverse        		; Start moving
    LOAD    Zero            		; Clear Remote code
    STORE   Remote          		; ""
EndWaitForward:
    RETURN                  		; Bail out

;*******************************************************************************
; WaitForForwardSmallBtn: moves forward two feet when zero is pressed
;*******************************************************************************
WaitForForwardSmallBtn:
    LOAD    Remote              ; Get remote code
    SUB     BtnForwardSmall     ; Subtract code for button forward
    JNEG    EndWaitForwardSmall ; If negative or positive end
    JPOS    EndWaitForwardSmall ; ""
    LOAD    DistBetweenParallel ; Distance to next parallel spot
    STORE   Distance            ; ""
    CALL    Traverse            ; Start moving
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
EndWaitForwardsmall:
    RETURN                      ; Bail out

;*******************************************************************************
; WaitForRightTurnBtn: turns left 90 degrees when fast forward is pressed
;*******************************************************************************
WaitForRightTurnBtn:
    LOAD    Remote              ; Get remote code
    SUB     BtnTurnRight        ; Subtract code for turn right button
    JNEG    EndWaitTurnRight    ; If negative or positive end
    JPOS    EndWaitTurnRight    ; ""
    LOADI   -87                 ; -90 Degrees
    STORE   Angle               ; ""
    CALL    Turn                ; Start turning
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
EndWaitTurnRight:
    RETURN                      ; Bail out

;*******************************************************************************
; WaitForRightTurnSmallBtn: turns left 5 degrees when fast forward is pressed
;*******************************************************************************
WaitForRightTurnSmallBtn:
    LOAD    Remote                  ; Get remote code
    SUB     BtnTurnRightSmall       ; Subtract code for turn right button
    JNEG    EndWaitTurnRightSmall   ; If negative or positive end
    JPOS    EndWaitTurnRightSmall   ; ""
    LOADI   -8                      ; -8 Degrees
    STORE   Angle                   ; ""
    CALL    Turn                    ; Start turning
    LOAD    Zero                    ; Clear Remote code
    STORE   Remote                  ; ""
EndWaitTurnRightSmall:
    RETURN                          ; Bail out

;*******************************************************************************
; WaitForLeftTurnBtn: turns left 90 degrees when rewind is pressed
;*******************************************************************************
WaitForLeftTurnBtn:
    LOAD    Remote              ; Get remote code
    SUB     BtnTurnLeft         ; Subtract code for button turn left
    JNEG    EndWaitTurnLeft     ; If negative or positive end
    JPOS    EndWaitTurnLeft     ; ""
    LOADI   87                  ; 90 Degrees
    STORE   Angle               ; ""
    CALL    Turn                ; Start turning
    LOAD    Zero                ; Clear Remote code
    STORE   Remote              ; ""
EndWaitTurnLeft:
    RETURN                      ; Bail out

;*******************************************************************************
; WaitForLeftTurnSmallBtn: turns left 5 degrees when rewind is pressed
;*******************************************************************************
WaitForLeftTurnSmallBtn:
    LOAD    Remote                  ; Get remote code
    SUB     BtnTurnLeftSmall        ; Subtract code for button turn left
    JNEG    EndWaitTurnLeftSmall    ; If negative or positive end
    JPOS    EndWaitTurnLeftSmall    ; ""
    LOADI   8                       ; 8 Degrees
    STORE   Angle                   ; ""
    CALL    Turn                    ; Start turning
    LOAD    Zero                    ; Clear Remote code
    STORE   Remote                  ; ""
EndWaitTurnLeftSmall:
    RETURN                          ; Bail out

;*******************************************************************************
; WaitFT: waits for the time stored in WaitTime
;*******************************************************************************
WaitFT:
    OUT     TIMER       ; Clear the timer
WFloop:
    IN      TIMER       ; Get time passed
    OUT     XLEDS       ; User-feedback that a pause is occurring.
    SUB     WaitTime    ; Subtract wait time.
    JNEG    WFloop      ; If its negative loop baby.
    LOAD    Zero        ; Clear WaitTime
    STORE   WaitTime    ; ""
    RETURN              ; Bail out

;*******************************************************************************
; Stop: stops the robot and resets odometry
;*******************************************************************************
Stop:
    LOAD    Zero        ; Load zero
    OUT     LVELCMD     ; Stop the robot
    OUT     RVELCMD     ; ""
    OUT     RESETPOS    ; Reset odometry
    IN      THETA       ; Pull in current heading
    STORE   Dtheta      ; Store it so we dont turn
    LOAD    Zero        ; Load Zero
    STORE   Dvel        ; No desired velocity
    Call    Wait1       ; Wait 1 second
    RETURN              ; Bail out

;*******************************************************************************
; Traverse: Moves forward by amount stored in Distance
;*******************************************************************************
Traverse:
    CALL    GetWheelEncoderDist ; Get both wheel encoder starting positions
    STORE   StartX              ; Store that bad boy
    IN      THETA               ; Dont change the angle cause we aint turnin'
    STORE   Dtheta              ; ""
    LOAD    FMid                ; Mid speed
    STORE   DVel                ; ""
    SEI     &B0010              ; Enable interrupts from source 2 (timer)
Onwards:
    CALL    GetWheelEncoderDist ; Get both wheel encoder starting positions
    SUB     StartX              ; Subtract the starting position
    SUB     Distance            ; Subtract the distance we want to go
    JNEG    Onwards             ; If negative repeat
    LOAD    Zero                ; Load zero
    STORE   DVel                ; Stop
    CALL    Wait1               ; Make sure the robot has stopped
    CALL    GetThetaErr         ; Get our angle error during traversal
    STORE   Angle               ; Make robot point proper direction
    CLI     &B0010              ; Disable interrupts from source 2 (timer)
    CALL    Turn                ; Initiate the angle correction
    LOAD    Zero                ; Clear Distance & Temp
    STORE   Distance            ; ""
    STORE   Temp                ; ""
    CALL    Stop                ; Stop the robot & reset odometry
    RETURN                      ; Bail out

;*******************************************************************************
; Turn: Turns the robot by the angle stored in Angle
;*******************************************************************************
Turn:
    LOAD    Zero        ; Store forward velocity as 0 cause we aint goin' straight
    STORE   DVel        ; ""
    IN      THETA       ; Get current theta
    ADD     Angle       ; Add our angle to current theta and store
    STORE   DTheta      ; ""
    SEI     &B0010      ; Enable interrupts from source 2 (timer)
TurnLoop:
    CALL    GetThetaErr ; Get the heading error
    CALL    Abs         ; Abolute value that shit
    ADDI    -1          ; Check if within 1 degrees
    JPOS    TurnLoop    ; If not, keep turning
    LOAD    Zero        ; Clear Angle
    STORE   Angle       ; ""
    CLI     &B0010      ; Disable interrupts from source 2 (timer)
    CALL    Stop        ; Stop the robot & reset odometry
    RETURN              ; Bail out


;*******************************************************************************
; WaitForRemote: loops forever until key stored in RemoteCheck is pressed then returns
;*******************************************************************************
WaitForRemote:
    IN     IR_LO            ; Get the low word
    SUB    RemoteCheck      ; Subtract key code
    JPOS   WaitForRemote    ; If positive loop
    JNEG   WaitForRemote    ; If negative loop
    OUT    IR_LO            ; Clear IR buffer for next check.
    RETURN                  ; Has to be zero so return

;*******************************************************************************
; Mod360: modulo 360
; Returns AC%360 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Mod360:
    ; easy modulo: subtract 360 until negative then add 360 until not negative
    JNEG   M360N
    ADDI   -360
    JUMP   Mod360
M360N:
    ADDI   360
    JNEG   M360N
    RETURN

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Neg: 2's complement negation
; Returns -AC in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
    JPOS   Abs_r
Neg:
    XOR    NegOne       ; Flip all bits
    ADDI   1            ; Add one (i.e. negate number)
Abs_r:
    RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW 0                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
    LOAD   AtanY
    CALL   Abs          ; abs(y)
    STORE  AtanT
    LOAD   AtanX        ; abs(x)
    CALL   Abs
    SUB    AtanT        ; abs(x) - abs(y)
    JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
    LOAD   AtanX        ; Octants 1, 4, 5, 8
    JNEG   A2_R3
    CALL   A2_calc      ; Octants 1, 8
    JNEG   A2_R1n
    RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
    ADDI   360          ; Add 360 if we are in octant 8
    RETURN
A2_R3: ; region 3
    CALL   A2_calc      ; Octants 4, 5
    ADDI   180          ; theta' = theta + 180
    RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7
    LOAD   AtanY        ; Swap input arguments
    STORE  AtanT
    LOAD   AtanX
    STORE  AtanY
    LOAD   AtanT
    STORE  AtanX
    JPOS   A2_R2        ; If Y positive, octants 2,3
    CALL   A2_calc      ; else octants 6, 7
    CALL   Neg          ; Negatge the number
    ADDI   270          ; theta' = 270 - theta
    RETURN
A2_R2: ; region 2
    CALL   A2_calc      ; Octants 2, 3
    CALL   Neg          ; negate the angle
    ADDI   90           ; theta' = 90 - theta
    RETURN
A2_calc:
    ; calculates R/(1 + 0.28125*R^2)
    LOAD   AtanY
    STORE  d16sN        ; Y in numerator
    LOAD   AtanX
    STORE  d16sD        ; X in denominator
    CALL   A2_div       ; divide
    LOAD   dres16sQ     ; get the quotient (remainder ignored)
    STORE  AtanRatio
    STORE  m16sA
    STORE  m16sB
    CALL   A2_mult      ; X^2
    STORE  m16sA
    LOAD   A2c
    STORE  m16sB
    CALL   A2_mult
    ADDI   256          ; 256/256+0.28125X^2
    STORE  d16sD
    LOAD   AtanRatio
    STORE  d16sN        ; Ratio in numerator
    CALL   A2_div       ; divide
    LOAD   dres16sQ     ; get the quotient (remainder ignored)
    STORE  m16sA        ; <= result in radians
    LOAD   A2cd         ; degree conversion factor
    STORE  m16sB
    CALL   A2_mult      ; convert to degrees
    STORE  AtanT
    SHIFT  -7           ; check 7th bit
    AND    One
    JZERO  A2_rdwn      ; round down
    LOAD   AtanT
    SHIFT  -8
    ADDI   1            ; round up
    RETURN
A2_rdwn:
    LOAD   AtanT
    SHIFT  -8           ; round down
    RETURN
A2_mult: ; multiply, and return bits 23..8 of result
    CALL   Mult16s
    LOAD   mres16sH
    SHIFT  8            ; move high word of result up 8 bits
    STORE  mres16sH
    LOAD   mres16sL
    SHIFT  -8           ; move low word of result down 8 bits
    AND    LowByte
    OR     mres16sH     ; combine high and low words of result
    RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
    LOADI  9            ; loop 8 times (256 = 2^8)
    STORE  AtanT
A2_DL:
    LOAD   AtanT
    ADDI   -1
    JPOS   A2_DN        ; not done; continue shifting
    CALL   Div16s       ; do the standard division
    RETURN
A2_DN:
    STORE  AtanT
    LOAD   d16sN        ; start by trying to scale the numerator
    SHIFT  1
    XOR    d16sN        ; if the sign changed,
    JNEG   A2_DD        ; switch to scaling the denominator
    XOR    d16sN        ; get back shifted version
    STORE  d16sN
    JUMP   A2_DL
A2_DD:
    LOAD   d16sD
    SHIFT  -1           ; have to scale denominator
    STORE  d16sD
    JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
    LOADI  0
    STORE  m16sc        ; clear carry
    STORE  mres16sH     ; clear result
    LOADI  16           ; load 16 to counter
Mult16s_loop:
    STORE  mcnt16s
    LOAD   m16sc        ; check the carry (from previous iteration)
    JZERO  Mult16s_noc  ; if no carry, move on
    LOAD   mres16sH     ; if a carry,
    ADD    m16sA        ;  add multiplicand to result H
    STORE  mres16sH
Mult16s_noc: ; no carry
    LOAD   m16sB
    AND    One          ; check bit 0 of multiplier
    STORE  m16sc        ; save as next carry
    JZERO  Mult16s_sh   ; if no carry, move on to shift
    LOAD   mres16sH     ; if bit 0 set,
    SUB    m16sA        ;  subtract multiplicand from result H
    STORE  mres16sH
Mult16s_sh:
    LOAD   m16sB
    SHIFT  -1           ; shift result L >>1
    AND    c7FFF        ; clear msb
    STORE  m16sB
    LOAD   mres16sH     ; load result H
    SHIFT  15           ; move lsb to msb
    OR     m16sB
    STORE  m16sB        ; result L now includes carry out from H
    LOAD   mres16sH
    SHIFT  -1
    STORE  mres16sH     ; shift result H >>1
    LOAD   mcnt16s
    ADDI   -1           ; check counter
    JPOS   Mult16s_loop ; need to iterate 16 times
    LOAD   m16sB
    STORE  mres16sL     ; multiplier and result L shared a word
    RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
    LOADI  0
    STORE  dres16sR     ; clear remainder result
    STORE  d16sC1       ; clear carry
    LOAD   d16sN
    XOR    d16sD
    STORE  d16sS        ; sign determination = N XOR D
    LOADI  17
    STORE  d16sT        ; preload counter with 17 (16+1)
    LOAD   d16sD
    CALL   Abs          ; take absolute value of denominator
    STORE  d16sD
    LOAD   d16sN
    CALL   Abs          ; take absolute value of numerator
    STORE  d16sN
Div16s_loop:
    LOAD   d16sN
    SHIFT  -15          ; get msb
    AND    One          ; only msb (because shift is arithmetic)
    STORE  d16sC2       ; store as carry
    LOAD   d16sN
    SHIFT  1            ; shift <<1
    OR     d16sC1       ; with carry
    STORE  d16sN
    LOAD   d16sT
    ADDI   -1           ; decrement counter
    JZERO  Div16s_sign  ; if finished looping, finalize result
    STORE  d16sT
    LOAD   dres16sR
    SHIFT  1            ; shift remainder
    OR     d16sC2       ; with carry from other shift
    SUB    d16sD        ; subtract denominator from remainder
    JNEG   Div16s_add   ; if negative, need to add it back
    STORE  dres16sR
    LOADI  1
    STORE  d16sC1       ; set carry
    JUMP   Div16s_loop
Div16s_add:
    ADD    d16sD        ; add denominator back in
    STORE  dres16sR
    LOADI  0
    STORE  d16sC1       ; clear carry
    JUMP   Div16s_loop
Div16s_sign:
    LOAD   d16sN
    STORE  dres16sQ     ; numerator was used to hold quotient result
    LOAD   d16sS        ; check the sign indicator
    JNEG   Div16s_neg
    RETURN
Div16s_neg:
    LOAD   dres16sQ     ; need to negate the result
    CALL   Neg
    STORE  dres16sQ
    RETURN
d16sN:      DW 0 ; numerator
d16sD:      DW 0 ; denominator
d16sS:      DW 0 ; sign value
d16sT:      DW 0 ; temp counter
d16sC1:     DW 0 ; carry value
d16sC2:     DW 0 ; carry value
dres16sQ:   DW 0 ; quotient result
dres16sR:   DW 0 ; remainder result

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
    ; take abs() of each value, and find the largest one
    LOAD   L2X
    CALL   Abs
    STORE  L2T1
    LOAD   L2Y
    CALL   Abs
    SUB    L2T1
    JNEG   GDSwap    ; swap if needed to get largest value in X
    ADD    L2T1
CalcDist:
    ; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
    STORE  m16sa
    LOADI  246       ; max * 246
    STORE  m16sB
    CALL   Mult16s
    LOAD   mres16sH
    SHIFT  8
    STORE  L2T2
    LOAD   mres16sL
    SHIFT  -8        ; / 256
    AND    LowByte
    OR     L2T2
    STORE  L2T3
    LOAD   L2T1
    STORE  m16sa
    LOADI  104       ; min * 104
    STORE  m16sB
    CALL   Mult16s
    LOAD   mres16sH
    SHIFT  8
    STORE  L2T2
    LOAD   mres16sL
    SHIFT  -8        ; / 256
    AND    LowByte
    OR     L2T2
    ADD    L2T3     ; sum
    RETURN
GDSwap: ; swaps the incoming X and Y
    ADD    L2T1
    STORE  L2T2
    LOAD   L2T1
    STORE  L2T3
    LOAD   L2T2
    STORE  L2T1
    LOAD   L2T3
    JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0


; Subroutine to wait (block) for 1 second
Wait1:
    OUT    TIMER
Wloop:
    IN     TIMER
    OUT    XLEDS       ; User-feedback that a pause is occurring.
    ADDI   -10         ; 1 second at 10Hz.
    JNEG   Wloop
    RETURN

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
    CALL   GetBattLvl
    JZERO  BattCheck   ; A/D hasn't had time to initialize
    SUB    MinBatt
    JNEG   DeadBatt
    ADD    MinBatt     ; get original value back
    RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
    LOADI  &H20
    OUT    BEEP        ; start beep sound
    CALL   GetBattLvl  ; get the battery level
    OUT    SSEG1       ; display it everywhere
    OUT    SSEG2
    OUT    LCD
    LOAD   Zero
    ADDI   -1          ; 0xFFFF
    OUT    LEDS        ; all LEDs on
    OUT    XLEDS
    CALL   Wait1       ; 1 second
    LOADI  &H140       ; short, high-pitched beep
    OUT    BEEP        ; stop beeping
    LOAD   Zero
    OUT    LEDS        ; LEDs off
    OUT    XLEDS
    CALL   Wait1       ; 1 second
    JUMP   DeadBatt    ; repeat forever

; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
    LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
    OUT    I2C_CMD     ; to I2C_CMD
    OUT    I2C_RDY     ; start the communication
    CALL   BlockI2C    ; wait for it to finish
    IN     I2C_DATA    ; get the returned data
    RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
    CALL   BlockI2C    ; wait for idle
    LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
    OUT    I2C_CMD     ; to I2C_CMD register
    LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
    OUT    I2C_DATA    ; to I2C_DATA register
    OUT    I2C_RDY     ; start the communication
    CALL   BlockI2C    ; wait for it to finish
    RETURN

; Subroutine to block until I2C device is idle
BlockI2C:
    LOAD   Zero
    STORE  Temp        ; Used to check for timeout
BI2CL:
    LOAD   Temp
    ADDI   1           ; this will result in ~0.1s timeout
    STORE  Temp
    JZERO  I2CError    ; Timeout occurred; error
    IN     I2C_RDY     ; Read busy signal
    JPOS   BI2CL       ; If not 0, try again
    RETURN             ; Else return
I2CError:
    LOAD   Zero
    ADDI   &H12C       ; "I2C"
    OUT    SSEG1
    OUT    SSEG2       ; display error message
    JUMP   I2CError

;***************************************************************
;* Variables
;***************************************************************
Temp:                   DW 0        ; "Temp" is not a great name, but can be useful
Distance:               DW 0        ; Distance to be traveled when moving
AutoDistance:           DW 0        ; Distance for automation
Angle:                  DW 0        ; Angle to turn to
AutoAngle:              DW 0        ; Angle for automation
StartX:                 DW 0        ; Starting X used for forward motion
RemoteCheck:            DW 0        ; Used for wait for remote and check button
Remote:                 DW 0        ; Used to temporarily store the remote code for loop
WaitTime:               DW 0        ; Used to wait for a certain amount of time
BtnForward:             DW 0        ; Button for moving forward
BtnTurnRight:           DW 0        ; Button for turning right
BtnTurnLeft:            DW 0        ; Button for turning left
BtnForwardSmall:        DW 0        ; Button for moving forward a small amount
BtnTurnRightSmall:      DW 0        ; Button for turning right a small amount
BtnTurnLeftSmall:       DW 0        ; Button for turning left a small amount
BtnParkParallel:        DW 0        ; Button for parallel parking
BtnParkPerp:        	DW 0        ; Button for perp parking
BtnParkPerpFirst:       DW 0        ; Button for perp parking getting to first
BtnForwardPerp:         DW 0        ; Button for moving to next perp spot
DistBetweenParallel:    DW &H294    ; Distance between parallel spots center to center
DistBetweenPerp:        DW &H1EA    ; Distance between perpindicular spots center to center
DistStartToParallel:    DW &H6C8    ; Distance to first parallel spot (center)
DistPerpOne:            DW &H2F0    ; Distance to first 90 turn for perpindicular
DistPerpTwo:            DW &H65E    ; Distance to second 90 turn for perpinducular
DistPerpThree:          DW &H0B0    ; Distance from second 90 turn to first perpindicular spot (center)
DistPerpFour:           DW &H14F	; Distance from second 90 turn to first perpindicular spot (center)
PerpSpace:              DW -1       ; Spot chosen for perpendicular autonomous parking
DistToTwo:              DW &H1EA    ; Distance to the second perp spot
DistToThree:            DW &H4E8    ; Distance to the second perp spot
DistToFour:             DW &H7E1    ; Distance to the second perp spot
DistToFive:             DW &HAE1    ; Distance to the second perp spot
DistToSix:              DW &HDC6    ; Distance to the second perp spot
DistToSeven:            DW &H108E   ; Distance to the second perp spot
;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

;***************************************************************
;* Remote Codes
;***************************************************************
Power:      DW &H00FF
VolUp:      DW &H40BF
VolDown:    DW &HC03F
Mute:       DW &H906F
ChanUp:     DW &H807F
ChanDown:   DW &H40BF
PrevChan:   DW &H42BD
BtnOne:     DW &H20DF
BtnTwo:     DW &HA05F
BtnThree:   DW &H609F
BtnFour:    DW &HE01F
BtnFive:    DW &H30CF
BtnSix:     DW &HB04F
BtnSeven:   DW &H708F
BtnEight:   DW &HF00F
BtnNine:    DW &H38C7
BtnZero:    DW &HB847
BtnEnter:   DW &H3AC5
TvVcr:      DW &HFF00
Rew:        DW &H48B7
Play:       DW &H28D7
FastF:      DW &HC837
BtnPause:   DW &H8877
BtnStop:    DW &H08F7
BtnRec:     DW &HA857

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter:   DW 961       ; ~1m in 1.04mm units
HalfMeter:  DW 481       ; ~0.5m in 1.04mm units
TwoFeet:    DW 586       ; ~2ft in 1.04mm units
Deg90:      DW 90        ; 90 degrees in odometer units
Deg180:     DW 180       ; 180
Deg270:     DW 270       ; 270
Deg360:     DW 360       ; can never actually happen; for math only
FSlow:      DW 100       ; 100 is about the lowest velocity value that will move
RSlow:      DW -100
FMid:       DW 350       ; 350 is a medium speed
RMid:       DW -350
FFast:      DW 500       ; 500 is almost max speed (511 is max)
RFast:      DW -500
MinBatt:    DW 140       ; 14.0V - minimum safe battery voltage
I2CWCmd:    DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:    DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
IR_HI:    EQU &HD0  ; read the high word of the IR receiver (OUT will clear both words)
IR_LO:    EQU &HD1  ; read the low word of the IR receiver (OUT will clear both words)
