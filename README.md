# Custom_SELV_PFC_System
This repository contains the hardware schematics, PCB layout, and software for a custom 24VAC Active Power Factor Correction (PFC) demonstrator. Operating entirely within the safe SELV (Safety Extra-Low Voltage) domain, the board is designed as a benchtop instrument for analyzing low-frequency AC metrology and compensating highly inductive loads.Controlled by a Raspberry Pi Pico 2 with full galvanic isolation, the system utilizes a high-speed ADC to calculate real, apparent, and reactive power on the fly. It dynamically corrects lagging power factors by switching a 4-channel binary-weighted capacitor array ($4.7\mu\text{F}$ to $44\mu\text{F}$) using zero-crossing synchronized triacs.

<img width="1420" height="780" alt="TOP" src="https://github.com/user-attachments/assets/dfe56269-5bf0-4e19-9cd6-821bc3d2eabb" />


<img width="1280" height="720" alt="BOTTOM" src="https://github.com/user-attachments/assets/13fcf632-e09b-4a31-a0ee-25804c42ef7d" />

