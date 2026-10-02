# Cogless firmware (CH32M007G8R6)

Sensorless six-step (trapezoidal) BLDC controller for the Cogless ESC. MounRiver Studio 2 project, WCH CH32V00X peripheral library.

## Files

| File | Role |
|---|---|
| `User/board.h` | Pin map and scaling constants, derived from `hardware/hardware.kicad_sch` |
| `User/motor_config.h` | Tunables: PWM frequency, dead time, start ramp, current and voltage limits |
| `User/motor.c` | 48 kHz control loop: TIM1 PWM to the internal gate driver, PGA current sense, BEMF zero-crossing commutation, protections |
| `User/rc_input.c` | Servo PWM throttle capture on J2.3 (EXTI on PA1, TIM2 as time base) |
| `User/telemetry.c` | Text telemetry over the SDI debug wire, non-blocking |
| `User/motor_tune.c` | Motor auto-detection, start-up adaptation, flash storage |
| `User/main.c` | 1 ms scheduler: arming, derating, LED, watchdog |

## Pin usage

| MCU pin | Function | Firmware |
|---|---|---|
| PA3, PB0, PB1 | HO1..3 (TIM1 CH1..3, remap 0100b) | High-side gates U, V, W |
| PA0, PA2, PD0 | LO1..3 (TIM1 CH1N..3N) | Low-side gates, complementary with 1 us dead time |
| PD7 / PA4 | OPA_P1 / OPA_N2 | Shunt, differential PGA x16, VDD/2 bias |
| PD2, PD3, PD4 | ADC_IN3, IN4, IN7 | Phase U, V, W back-EMF dividers |
| PD5 | ADC_IN5 | Battery voltage |
| PD6 | ADC_IN6 | NTC (Cantherm CMFA103J3500HANT, 10k B3500) |
| PA1 | EXTI1 + TIM2 free counter | Servo PWM input, 1000..2000 us (TIM2 remaps 5/6 would steal PA0/PA2/PA3 from TIM1) |
| PC4 | GPIO | Status LED |
| PC2 | SWIO | WCH-LinkE, also carries telemetry |

## Status

Validated on the bench on 2026-09-30 with a 1503 motor on 3S: open-loop start, hand-off to sensorless closed loop, stable run around 3 500 rpm (10 % duty). Current sensing, hardware over-current break and software limits are active.

## How it works

Each PWM period (48 kHz, centre-aligned) TIM1 CH4 starts an ADC injected sequence inside the high-side ON pulse: PGA output (current), floating-phase voltage, bus voltage. The floating phase is also sampled in the middle of the OFF time, where it reads 1.5 times its back-EMF against ground; that sample drives the zero-crossing detector. The loop applies blanking after each commutation and commutates 30 degrees (minus a small advance) after the crossing. When the first samples show the crossing already passed, the crossing is dated at the detection, so a rotor running ahead is caught up. Startup: bootstrap precharge (all low-side on), alignment, then an open-loop ramp until six consecutive zero crossings land in the middle of the forced steps.

Protections:

- Hardware over-current: PGA output into CMP2 into the TIM1 break input, about 66 A, no software involved.
- Software over-current at 45 A (3 samples), current fold-back above 30 A.
- Desync (no zero crossing for 60 ms or 4 periods) and stall (ramp timeout): outputs off, automatic retry after 500 ms at zero throttle.
- Under-voltage: fold-back below 3.3 V/cell, stop below 3.0 V/cell. Cell count detected at arming (2S, 3S, 4S).
- Over-temperature: fold-back above 80 C, stop at 100 C. The NTC is read every 100 ms, also while the motor runs.
- RC failsafe: no pulse for 100 ms stops the motor and disarms.
- Independent watchdog, 100 ms.

## Arming and LED

1. Power on with the throttle at minimum. The LED gives one long blink (500 ms) every 2 s while no RC signal is seen, a slow blink (1 s on, 1 s off) while waiting.
2. Hold zero throttle for 1 s: LED solid, armed.
3. Throttle up: LED blinks at 4 Hz while running.
4. Fault: the LED blinks the fault code, then pauses. 1 hardware over-current, 2 software over-current, 3 desync, 4 stall, 5 under-voltage, 6 over-temperature, 7 control-loop overrun.

## Telemetry

Connect a WCH-LinkE and open the SDI printf console in MounRiver (or `wlink`). A line is printed every 200 ms:

```
st=4 f=0 arm=1 rc=1500 thr=500 duty=520 erpm=24000 I=6200mA V=15800mV 4S T=41C
```

Without a debugger the writer times out on the first packet and stays silent, so the control loop is never blocked.

## First bring-up checklist

1. JP1 must be closed for every supported pack (the internal regulator needs more than 18 V, see the datasheet), whatever the silkscreen says.
2. Use a current-limited supply, no propeller.
3. Check with a scope that the LO/HO outputs show complementary PWM with dead time on `motor_start()`.
4. If the motor twitches and stops with fault 4 (stall), tune `RAMP_DUTY_*` and `RAMP_*_STEP_US` in `motor_config.h` for the motor. Small high-KV motors need shorter steps; large motors need more duty.
5. Direction: build with `-DMOTOR_REVERSE=1` or swap two motor wires.

## Motor auto-detection (`User/motor_tune.c`)

At power-up, when no result is stored in flash (`AUTOTUNE_MODE 1`), the controller characterises the motor, **which must turn freely, without propeller**:

1. **Standstill sweep** on one step: the duty that gives `TUNE_I_TARGET_MA`, and a phase-to-phase resistance estimate.
2. **Parasitic floor**: the floating-phase reading in the PWM OFF time with that current flowing.
3. **Open-loop acceleration** at regulated current, 20 to 670 steps/s. The BEMF peak is recorded at every step. Synchronism is checked with peak x period, which stays constant while the rotor follows. Loss of sync triggers a retry with a gentler ramp, then with 1.5 x the current (6 attempts).
4. **Hand-off speed**: where the BEMF peak reaches 4 x the floor (at least 40 LSB).
5. **Storage** in the last 256 B of flash (0x0800F700, excluded in `Ld/Link.ld`). The result is reloaded at every power-up.

LED: fast blinking during the tune, three long blinks when done. If the tune fails, the LED blinks the error code twice (3 = no motor, 4 = over-current, 5 = no synchronism, 6 = flash) and the defaults stay in use. The details are in `tune_res` (debugger). Reflashing the firmware usually erases the record, so the tune runs again. `AUTOTUNE_MODE 2` forces a tune at every power-up.

## Bring-up findings (keep these)

- **TIM2 remap for the RC input breaks the gate driver.** Remaps 5/6 put TIM2 channels on PA0/PA2/PA3 and steal them from TIM1. The RC input uses EXTI on PA1 instead.
- **Off phases must keep their driver inputs driven.** With CCxE = CCxNE = 0, TIM1 releases both pins and the CH283 inputs float. Off phases use CCxE = 1 with the output forced inactive.
- **OCxN only follows OCxREF on an edge.** Switching a channel straight to "forced inactive" can leave its low side off. `apply_regs()` inserts a pulse shorter than the dead time to create the edge.
- **The shunt only carries current during the high-side pulse.** It is sampled first in the ADC sequence, right at the centre of the pulse.
- **Never disable the CMP2 break.** Its trips were real current; a 1503 burned while it was off.
- **The ADC sample time is per channel.** U and V BEMF channels left at reset kept ~9 % of the previous conversion (160 LSB on a phase at 0 V). Every channel is now set explicitly, and the ADC leaves low-power mode.
- **The BEMF is read in the PWM OFF time** (TIM1 update at the top of the count). The two driven phases are at 0 V, so there is no neutral to estimate.
- **Unloaded open loop leaves the rotor ahead of the commutation.** The zero crossing happens before the step starts, so an "already crossed" first sample is accepted at the end of the ramp, with the closed-loop acceleration limited per step.

## Bring-up switches (`motor_config.h`)

| Switch | Use |
|---|---|
| `TEST_AUTORUN 1` | runs without RC: arm, 10 s ON, 3 s OFF, forever. Set to 0 for RC control |
| `MOTOR_TEST_OPEN_LOOP 1` | forced six-step only, no sensorless hand-off |
| `DRIVER_PIN_TEST 1..6` | static probes of the gate driver, the PGA path and each phase; 0 for normal firmware |
| `motor_trace[]` | last 96 commutations: period, duty, current, zero-crossing position, BEMF extremes, faults |

## Limits of this version

- Zero crossings are sampled once per PWM period and the step is capped at 6 ticks, so the electrical speed is limited to about 80 000 eRPM at 48 kHz (about 11 400 rpm on a 14-pole motor). A proportional duty cap holds the speed just under that ceiling. At 48 kHz the OFF-time sample limits the duty to about 83 %.
- Servo PWM input only. DShot and the analog encoder input on PA1 are not implemented.
- No FOC yet: this is the six-step baseline to validate the hardware.

## Building outside MounRiver

The toolchain shipped with MounRiver Studio 2 works from the command line:

```
GCC="/Applications/MounRiver Studio 2.app/Contents/Resources/app/resources/darwin/components/WCH/Toolchain/RISC-V Embedded GCC12/bin/riscv-wch-elf-gcc"
CF="-march=rv32ec_zmmul -mabi=ilp32e -msmall-data-limit=8 -mno-save-restore -Os -fsigned-char -ffunction-sections -fdata-sections -fno-common -IStartup -IDebug -ICore -IUser -IPeripheral/inc"
for f in User/*.c Debug/*.c Core/*.c Peripheral/src/*.c; do "$GCC" $CF -c $f -o build/$(basename ${f%.c}).o; done
"$GCC" $CF -x assembler-with-cpp -c Startup/startup_ch32v00X.S -o build/startup.o
"$GCC" $CF -T Ld/Link.ld -nostartfiles -Xlinker --gc-sections --specs=nano.specs --specs=nosys.specs build/*.o -o build/fw.elf
```

## License

The firmware files written for Cogless (`User/` files with the Cogless SPDX header) are dual-licensed: [GPL-3.0-only](../LICENSE-FIRMWARE) or a commercial licence ([LICENSE-COMMERCIAL.md](../LICENSE-COMMERCIAL.md)). The WCH SDK files keep WCH's own terms. See the [root README](../README.md#license).
