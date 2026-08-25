# MicroMod Carrier V1.1 Board Reference

This is the firmware-facing hardware contract for the MicroMod Carrier. It
documents every carrier-connected MicroMod function and every exposed board
interface. Pin names describe the MicroMod standard signal; firmware must map
them to the GPIO names used by the installed MicroMod processor board.

The KiCad schematics are authoritative if this file and the hardware ever
disagree. In particular, confirm that the selected processor board implements
every MicroMod function used by the intended firmware.

## Electrical Rules

- All logic signals on J7, J8, J9, J6, and the NeoPixel data input use a 3.3 V
  domain. Do not apply 5 V to any of these signals.
- The battery input is 6-13 V. `VIN_PROTECTED` and J9 pin 4 can reach 13 V;
  they are not logic-level signals.
- `+5V_SYS` is the fused, muxed system rail. The resettable fuse is rated for
  1 A hold and 2 A trip.
- DYNAMIXEL power can be a switched protected battery rail or `+5V_SYS`,
  selected by solder jumper JP1. Inspect the jumper before enabling servos.
- RESET and BOOT are active-low pushbuttons to ground. They are not general
  GPIOs.
- The USB-C connector is a USB 2.0 device/sink port only. It is not a USB host
  or USB Power Delivery port.

## MicroMod Function Map

The table lists every functional MicroMod contact connected by the carrier.
Unlisted contacts are not application I/O on this board (they are unused,
reserved, or ground contacts defined by the MicroMod standard).

| Contact | MicroMod function | Carrier connection | Firmware use / constraint |
| ---: | --- | --- | --- |
| 2, 74 | 3.3 V | `+3V3` processor supply | Supply input; not a GPIO |
| 3 | USB D+ | USB-C J2 through ESD and a 0-ohm link | USB device data |
| 4 | 3.3V_EN | Not connected | The carrier 3.3 V regulator is always enabled |
| 5 | USB D- | USB-C J2 through ESD and a 0-ohm link | USB device data |
| 6 | RESET | RESET pushbutton | Active low; do not use as GPIO |
| 9 | USB_VIN | `+5V_SYS` | Board is powered from muxed battery/USB 5 V |
| 10 | D0 | EYESPI D/C | Display data/command control |
| 11 | BOOT | BOOT pushbutton | Active low; processor-specific boot function |
| 12 | I2C SDA | J7 and EYESPI J6 | Shared I2C bus, 3.3 V |
| 14 | I2C SCL | J7 and EYESPI J6 | Shared I2C bus, 3.3 V |
| 16 | I2C_INT | EYESPI J6 pin 4 | Capacitive-touch interrupt when fitted |
| 17 | UART1 TX | J8 pin 4 | 3.3 V UART output, named from processor perspective |
| 18 | D1 / CAM_TRIG | EYESPI reset | Display reset control |
| 19 | UART1 RX | J8 pin 3 | 3.3 V UART input, named from processor perspective |
| 20 | UART2 RX | DYNAMIXEL half-duplex receiver | Receive side of the TTL bus |
| 22 | UART2 TX | DYNAMIXEL half-duplex transmitter | Transmit side of the TTL bus |
| 32 | PWM0 | EYESPI `LITE` | Display backlight PWM through 22 ohms |
| 34 | A0 | `BAT_SENSE` | Reserved battery-voltage measurement input |
| 38 | A1 | J9 pin 10 | Expansion analog/GPIO, 3.3 V maximum |
| 40 | G0 | EYESPI SD chip select | Dedicated to EYESPI while J6 is in use |
| 41 | CAN RX | J9 pin 5 | CAN-capable GPIO input; no CAN transceiver is fitted |
| 42 | G1 | EYESPI touch chip select | Dedicated to EYESPI while J6 is in use |
| 43 | CAN TX | J9 pin 6 | CAN-capable GPIO output; no CAN transceiver is fitted |
| 44 | G2 | DYNAMIXEL `DXL_TXEN` | High: transmit. Low: receive/default-safe state |
| 46 | G3 | DYNAMIXEL high-side power switch | High enables switched servo power; low disables it |
| 47 | PWM1 | Onboard NeoPixel data | PWM/GPIO output through 33 ohms |
| 48 | G4 / BUS4 | EYESPI memory chip select | Dedicated to EYESPI while J6 is in use |
| 55 | SPI CS | EYESPI display chip select | Dedicated display CS |
| 57 | SPI SCK | EYESPI J6 pin 15 | Shared display/SD/memory/touch SPI clock |
| 59 | SPI SDO | EYESPI J6 pin 14 | SPI MOSI/COPI from the processor |
| 61 | SPI SDI | EYESPI J6 pin 13 | SPI MISO/CIPO into the processor |
| 67 | G8 | J9 pin 7 | Expansion GPIO, 3.3 V |
| 69 | G7 / BUS7 | J9 pin 8 | Expansion GPIO/bus signal, 3.3 V |
| 71 | G6 / BUS6 | J9 pin 9 | Expansion GPIO/bus signal, 3.3 V |
| 73 | G5 / BUS5 | EYESPI BUSY | Display busy input |

Contact 49 (`BATT_VIN`) is intentionally not connected. The carrier battery
rail can reach 13 V and must not be assumed safe for a universal processor
module input.

## Arduino Pin Map by Processor Board

Use the numbers below with Arduino functions such as `pinMode()`,
`digitalWrite()`, and `analogRead()`. The names in parentheses are standard
Arduino bus objects or aliases where applicable. `NC` means that the processor
board does not connect that MicroMod contact; software pin routing cannot make
the carrier signal available.

| Contact | Carrier / MicroMod signal | MicroMod Teensy Arduino | MicroMod ESP32 Arduino |
| ---: | --- | --- | --- |
| 10 | EYESPI D/C / D0 | pin 4 | GPIO14 |
| 12 | I2C SDA | pin 18 (`SDA`, `Wire`) | GPIO21 (`SDA`, `Wire`) |
| 14 | I2C SCL | pin 19 (`SCL`, `Wire`) | GPIO22 (`SCL`, `Wire`) |
| 16 | EYESPI interrupt / I2C_INT | pin 29 | GPIO4 |
| 17 | J8 UART1 TX | pin 1 (`Serial1` TX) | GPIO17 (`Serial1` TX) |
| 18 | EYESPI reset / D1 | pin 5 | GPIO27 |
| 19 | J8 UART1 RX | pin 0 (`Serial1` RX) | GPIO16 (`Serial1` RX) |
| 20 | DYNAMIXEL UART2 RX | pin 16 (`Serial2` RX) | NC |
| 22 | DYNAMIXEL UART2 TX | pin 17 (`Serial2` TX) | NC |
| 32 | EYESPI LITE / PWM0 | pin 3 | GPIO13 |
| 34 | BAT_SENSE / A0 | pin 14 (`A0`) | GPIO34, input only |
| 38 | J9 pin 10 / A1 | pin 15 (`A1`) | GPIO35, input only |
| 40 | EYESPI SD CS / G0 | pin 40 | GPIO15 |
| 41 | J9 CAN RX | pin 30 (`CAN1` RX) | NC |
| 42 | EYESPI touch CS / G1 | pin 41 | GPIO25 |
| 43 | J9 CAN TX | pin 31 (`CAN1` TX) | NC |
| 44 | DXL_TXEN / G2 | pin 42 | GPIO26 |
| 46 | DYNAMIXEL power enable / G3 | pin 43 | GPIO17 |
| 47 | NeoPixel / PWM1 | pin 2 | GPIO12 |
| 48 | EYESPI memory CS / G4 | pin 44 | GPIO16 |
| 55 | EYESPI display CS / SPI CS | pin 10 (`SS`) | GPIO5 (`SS`) |
| 57 | EYESPI SPI SCK | pin 13 (`SCK`, `SPI`) | GPIO18 (`SCK`, `SPI`) |
| 59 | EYESPI MOSI / SPI SDO | pin 11 (`MOSI`, `SPI`) | GPIO23 (`MOSI`, `SPI`) |
| 61 | EYESPI MISO / SPI SDI | pin 12 (`MISO`, `SPI`) | GPIO19 (`MISO`, `SPI`) |
| 67 | J9 pin 7 / G8 | pin 26 (`A12`, input only as analog) | NC |
| 69 | J9 pin 8 / G7 | pin 9 | NC |
| 71 | J9 pin 9 / G6 | pin 6 | GPIO33 |
| 73 | EYESPI BUSY / G5 | pin 45 | GPIO32 |

Supply, USB, reset, and boot contacts are omitted because they are not normal
Arduino application GPIOs. On Teensy, `LED_BUILTIN` is pin 13 and therefore
shares the EYESPI SPI clock. On ESP32, the processor-board status LED is GPIO2
and is separate from the carrier NeoPixel.

### ESP32 Limitations

- The DYNAMIXEL interface is not usable as routed: UART2 RX/TX contacts 20 and
  22 are not connected on the MicroMod ESP32 Processor Board. G2 and G3 remain
  available, but they provide only direction and power control without data.
- J9 CAN RX/TX, G7, and G8 are also unavailable because their MicroMod contacts
  are not connected by the ESP32 processor board.
- A0/GPIO34 and A1/GPIO35 are input-only. They have no internal pull-up or
  pull-down resistors.
- PWM1/GPIO12 and G0/GPIO15 are ESP32 boot-strapping pins. Avoid external
  circuits that force GPIO12 high or GPIO15 low while the processor resets.
- G3/GPIO17 and G4/GPIO16 share the pins used by `Serial1`. On this carrier,
  enabling DYNAMIXEL power with G3 conflicts with J8 UART1 transmission, and
  EYESPI memory CS on G4 conflicts with J8 UART1 reception.

## Connectors and Exposed Pins

### J2: USB-C

| Pin group | Connection |
| --- | --- |
| VBUS | `VBUS`, backup input to the 5 V power mux |
| D+ / D- | MicroMod USB D+ / D- through ESD protection |
| CC1 / CC2 | 5.1 kohm pull-downs; requests default USB power as a sink/device |
| SBU | Not connected |

### J3: Battery Input

| Pin | Net | Meaning |
| ---: | --- | --- |
| 1 | `VIN_RAW` | Positive battery input, 6-13 V |
| 2 | GND | Battery return |

`VIN_RAW` passes through reverse-polarity protection before becoming
`VIN_PROTECTED`. The buck regulator supplies 5 V from the protected rail; the
TPS2116 mux gives this battery-derived rail priority over USB VBUS.

### J4 and J5: DYNAMIXEL TTL

J4 and J5 are electrically parallel three-pin Molex `22-05-7035` connectors.

| Pin | Net | Meaning |
| ---: | --- | --- |
| 1 | `DXL_TTL` | Shared half-duplex TTL data |
| 2 | `DXL_VDD` | Selected servo supply |
| 3 | GND | Servo return |

Rev A errata: pins 1 and 3 were swapped (GND on pin 1, data on pin 3),
reversed relative to the actual DYNAMIXEL harness orientation. Rev A boards
need reworked/flipped cables; `DXL_VDD` stays on the center pin either way,
so a reversed harness swaps only data and ground.

JP1 selects the source for `DXL_VDD`:

| JP1 bridge | Source | Intended use |
| --- | --- | --- |
| Pads 1-2 (fabricated default) | Switched `VIN_PROTECTED` / `POWER_BUS` | Battery-powered DYNAMIXEL servos |
| Pads 2-3 | `+5V_SYS` | 5 V bench operation |

Never bridge both JP1 sides. That would tie the battery rail to `+5V_SYS`.

### J6: EYESPI 18-Pin FPC

J6 is a Hirose `FH12-18S-0.5SH(55)` EYESPI connector. The EYESPI FFC is an
A-B type cable (contacts on opposite sides at each end), which mirrors the
pin order end-to-end: carrier pin `n` mates with display pin `19 - n`. The
carrier-side assignment below is therefore the reverse of the display-side
EYESPI numbering. All active-low display chip-select and reset lines have
connector-side 100 kohm pull-ups, so they remain inactive while the processor
is reset or absent.

Rev A errata: J6 was assigned in display order (3V3 on pin 1), which only
mates through an A-A cable. Rev B reverses the order as tabulated here.

| Pin | EYESPI signal | MicroMod function / notes |
| ---: | --- | --- |
| 1 | GP2 | Not connected |
| 2 | GP1 | Not connected |
| 3 | BUSY | G5, display busy input |
| 4 | INT | I2C_INT, touch interrupt |
| 5 | SDA | Shared I2C SDA |
| 6 | SCL | Shared I2C SCL |
| 7 | TSCS | G1, touch-controller chip select |
| 8 | MEMCS | G4, onboard-memory chip select |
| 9 | SDCS | G0, SD card chip select |
| 10 | TCS | Primary SPI CS, display chip select |
| 11 | RESET | D1, display reset |
| 12 | DC | D0, display data/command |
| 13 | MISO | Primary SPI SDI, CIPO into the processor |
| 14 | MOSI | Primary SPI SDO, COPI from the processor |
| 15 | SCK | Primary SPI SCK |
| 16 | GND | Ground |
| 17 | LITE | PWM0 backlight control |
| 18 | 3V3 | `+3V3` supply |

### J7: Qwiic / I2C

J7 is a side-entry JST-SH `SM04B-SRSS-TB` connector.

| Pin | Net | Meaning |
| ---: | --- | --- |
| 1 | GND | Ground |
| 2 | `+3V3` | Logic supply |
| 3 | `I2C_SDA` | Shared I2C SDA |
| 4 | `I2C_SCL` | Shared I2C SCL |

The board provides 4.7 kohm I2C pull-ups. Optional companion pull-ups may be
removed when downstream devices already make the bus too strong.

### J8: 3.3 V UART1

J8 is a side-entry JST-SH `SM04B-SRSS-TB` connector. TX and RX names are from
the carrier processor perspective.

| Pin | Net | Meaning |
| ---: | --- | --- |
| 1 | GND | Ground |
| 2 | `+3V3` | Logic supply |
| 3 | UART1 RX | Data into the processor |
| 4 | UART1 TX | Data from the processor |

### J9: Expansion Header

J9 is a vertical 1x10, 2.54 mm header.

| Pin | Net | Meaning |
| ---: | --- | --- |
| 1 | GND | Common return |
| 2 | `+3V3` | Regulated logic rail |
| 3 | `+5V_SYS` | Fused, muxed 5 V rail |
| 4 | `VIN_PROTECTED` | Reverse-protected battery rail, 13 V maximum |
| 5 | CAN RX | MicroMod CAN-capable GPIO input |
| 6 | CAN TX | MicroMod CAN-capable GPIO output |
| 7 | G8 | General-purpose 3.3 V GPIO |
| 8 | G7 / BUS7 | General-purpose 3.3 V GPIO/bus signal |
| 9 | G6 / BUS6 | General-purpose 3.3 V GPIO/bus signal |
| 10 | A1 | Analog-capable 3.3 V GPIO |

J9 exposes CAN-capable processor pins only; it does not include a CAN
transceiver or a differential CAN bus connector.

## Onboard Controls and Indicators

| Item | Processor connection | Firmware notes |
| --- | --- | --- |
| SW1 RESET | RESET, contact 6 | Active-low reset pushbutton |
| SW2 BOOT | BOOT, contact 11 | Active-low bootloader pushbutton |
| D3 NeoPixel | PWM1, contact 47 | Data via 33 ohms; its 3.3 V supply is below the LED's 3.5 V specified minimum, so validate on hardware |
| TP1 | `VIN_PROTECTED` | Protected battery test point |
| TP2 | `+5V_SYS` | System 5 V test point |
| TP3 | `+3V3` | Logic supply test point |
| TP4 | GND | Ground test point |
| TP5 | `DXL_VDD` | DYNAMIXEL supply test point |
| TP6 | `DXL_TTL` | DYNAMIXEL data test point |
| TP7 | RESET | Reset test point |
| TP8 | BOOT | Boot test point |
| TP9 | USB D+ | USB data test point |
| TP10 | USB D- | USB data test point |

## DYNAMIXEL Firmware Contract

The DYNAMIXEL interface uses UART2 plus a hardware-controlled half-duplex
transceiver. Keep `DXL_TXEN` low except while transmitting.

| State | G2 / `DXL_TXEN` | UART2 behavior |
| --- | --- | --- |
| Idle / receive | Low | Transmitter disabled, receiver enabled |
| Transmit | High | Transmitter enabled; write the UART2 frame, wait for final-byte completion, then return low |

Use G3 to control servo power independently:

| State | G3 | Result |
| --- | --- | --- |
| Disabled / startup-safe | Low | DYNAMIXEL high-side switch off |
| Enabled | High | Selected `DXL_VDD` source applied to J4/J5 pin 2 |

Set G3 low and G2 low early during startup. Only enable the rail after the
JP1 source and connected servo voltage are known to be safe.

## Battery Measurement

MicroMod A0 receives `BAT_SENSE` through a 100 kohm / 27 kohm divider from
`VIN_PROTECTED`, filtered with 100 nF. Use the processor board's ADC reference
and calibration, then recover the protected rail as:

$$
V_{VIN\_PROTECTED} = V_{ADC} \times \frac{127}{27} \approx V_{ADC} \times 4.7037
$$

This measures the protected rail after the reverse-polarity MOSFET. The actual
battery terminal voltage is slightly higher under load by the MOSFET's voltage
drop. At the 13 V input limit, `BAT_SENSE` is nominally 2.764 V.

## Resource Conflicts and Initialization

- I2C SDA/SCL are shared by J7 and EYESPI. A display with capacitive touch may
  also use `I2C_INT`.
- The primary SPI bus is shared by the EYESPI display, SD card, onboard memory,
  and touch controller. Keep every unused chip select inactive before using a
  selected device.
- D0, D1, G0, G1, G4, G5, PWM0, SPI CS/SCK/SDI/SDO, and I2C_INT are consumed
  by EYESPI when J6 is used.
- UART2, G2, and G3 are consumed by DYNAMIXEL. UART1 remains available only at
  J8.
- A0 is dedicated to battery measurement. Use A1, G6, G7, and G8 on J9 for
  general expansion where the selected MicroMod supports them.
- CAN RX/TX on J9 require an external CAN transceiver before they can connect
  to a CAN bus.

Recommended startup states:

1. Configure G3 (`DXL` power enable) low and G2 (`DXL_TXEN`) low.
2. Configure all EYESPI chip-select and reset outputs inactive before starting
   SPI traffic.
3. Start I2C with 3.3 V-compatible devices only.
4. Enable DYNAMIXEL power only after confirming the JP1 rail selection.

## Power Architecture

```text
Battery (J3, 6-13 V) -> VIN_RAW -> reverse protection -> VIN_PROTECTED
                                           |
                                           +-> switched DYNAMIXEL supply -> JP1 -> DXL_VDD
                                           |
                                           +-> 5 V buck --+
USB-C VBUS -------------------------------+--------------> 5 V power mux -> fuse -> +5V_SYS -> 3.3 V regulator
```

The power mux uses the battery-derived 5 V rail as its preferred source when
valid; USB VBUS is the backup source. MicroMod `USB_VIN` is powered by
`+5V_SYS`, so the processor remains powered from either supported source.

## Source Schematics

| Schematic | Coverage |
| --- | --- |
| `01_MicroMod_USB.kicad_sch` | MicroMod socket, USB-C, RESET, BOOT |
| `02_Power.kicad_sch` | Battery, power mux, 5 V buck, 3.3 V regulator, A0 divider |
| `03_Dynamixel.kicad_sch` | J4/J5, JP1, servo power switch, UART2 half-duplex interface |
| `04_EYESPI.kicad_sch` | J6 display, SD, memory, touch, I2C, and backlight |
| `05_IO_Status.kicad_sch` | J7/J8/J9, NeoPixel, and test points |
*** End Patch