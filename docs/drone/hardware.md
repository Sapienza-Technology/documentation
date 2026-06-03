# Hardware

!!! warning "Frame attuale — telaio temporaneo"
    Il drone monta attualmente il **telaio vecchio** (TBS Source One V5 7").
    È in corso la progettazione di un nuovo telaio custom. Fino alla sua realizzazione, tutta la documentazione si riferisce al setup attuale.

---

## Frame

| Parameter | Value |
|-----------|-------|
| Model | TBS Source One V5 |
| Size | 7" |
| Material | Carbon fiber, aluminum standoffs, M3 screws |
| FC mount | 30.5×30.5mm (standard) |

!!! note "FC mount mismatch"
    The Pixhawk 6C and Raspberry Pi are too large for the standard 30.5×30.5mm mount.
    Both are mounted **underneath the frame** with a vibration damping system (rubber grommets).
    The frame has been raised ~8cm with extended legs to accommodate the new electronics stack.

---

## Motors

| Parameter | Value |
|-----------|-------|
| Model | Emax ECO II 2807 |
| KV | 1300 KV |
| Count | 4 |
| Configuration | Props out |

These motors are confirmed final — will not change with the new frame.

---

## Propellers

| Parameter | Value |
|-----------|-------|
| Model | HQProp 7×3.5×3 |
| Size | 7" |
| Blades | 3 |

These propellers are confirmed final — will not change with the new frame.

---

## Battery

!!! warning "Battery upgrade in evaluation"
    Currently using 4S LiPo batteries. A custom 6S Li-ion pack is under evaluation.
    **All electronics are rated for 6S** — the current 4S setup might be temporary.

=== "3Ah LiPo (current — lighter)"

    | Parameter | Value |
    |-----------|-------|
    | Chemistry | LiPo |
    | Cell count | 4S |
    | Capacity | 3000 mAh |
    | Total weight with drone | ~1.8 kg |

=== "5Ah LiPo (current — longer flight)"

    | Parameter | Value |
    |-----------|-------|
    | Chemistry | LiPo |
    | Cell count | 4S |
    | Capacity | 5000 mAh |
    | Total weight with drone | ~2.0 kg |

=== "Custom Li-ion 6S (in evaluation)"

    | Parameter | Value |
    |-----------|-------|
    | Chemistry | Li-ion |
    | Cells | Molicel P45B |
    | Configuration | 6S 1P |
    | Status | Under evaluation — not yet built |

---

## Weight

| Configuration | Weight |
|---------------|--------|
| With 3Ah LiPo | ~1.8 kg |
| With 5Ah LiPo | ~2.0 kg |

---

## Component layout

``` mermaid
graph TD
    subgraph TOP
        GPS[GPS\nraised on carbon rod]
    end
    subgraph UNDERFRAME
        subgraph FRONT
            RS[RealSense D435i\nvibration damped]
            VL[VL53L1X\nfront-right arm]
        end
        subgraph CENTER
            ESC[ESC SpeedyBee\ncenter frame]
            PX[Pixhawk 6C\nvibration damped]
            RPI[Raspberry Pi 5\nupside down]
        end
        subgraph REAR
            ESP[ESP8266\nrear-left arm]
            ELRS[ELRS receiver\nrear-right arm]
        end
    end
```

!!! tip "GPS placement"
    The GPS is mounted on a small carbon fiber rod, raised above the frame to reduce interference from other electronics. Secured with zip ties.

---

## Previous setup

The drone previously used a **Kakute H7** flight controller on the same frame.
The Kakute H7 has been replaced by the Pixhawk 6C as part of the current setup upgrade.

---

## TODO

- [ ] **Design and build new custom frame** — the TBS Source One V5 is a temporary solution. The new frame must accommodate the Pixhawk 6C, Raspberry Pi 5, and all peripherals without the current workarounds (extended legs, underframe mounting). This is a major ongoing task.
- [ ] Finalize battery choice — evaluate and build custom 6S Li-ion pack (Molicel P45B, 6S 1P)
- [ ] Document vibration damping system in detail (grommet specs, mount design)
- [ ] Document new frame design once complete

---

*Maintainer: Alessandro Paolantonio — for questions or issues, contact Alessandro.*