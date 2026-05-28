# Universal Bearingless Motor Model and Controller

## Introduction

This simulation package includes all necessary files to simulate any bearingless motor with a combined winding. The model was originally developed in the following publication and includes both a universal controller and plant:

```markdown
Takahiro NOGUCHI, Mohamadhasan MOKHTARABADI, Kamisetti N V PRASAD, Wolfgang GRUBER and Eric L. SEVERSON,
"Model and Control Framework for Bearingless Motors with Combined Windings"
19th International Symposium on Magnetic Bearings (ISMB19), 2025.
```

The machine parameters were subsequently updated and are under review for publication as the following article:

```markdown
Takahiro NOGUCHI, Mohamadhasan MOKHTARABADI, Kamisetti N V PRASAD, David PRINZ, Wolfgang GRUBER and Eric L. SEVERSON,
"Universal Model and Control Framework for Bearingless Motors with Combined Windings"
Actuators, 2026.
```

## Folder Contents

### [`setup_ismb_19.m`](./setup_ismb_19.m)

This MATLAB script defines the machine parameters and runs the Simulink simulation to reproduce the results shown in Fig. 6 of the ISMB19 publication.

### [`setup_actuator_2026.m`](./setup_actuator_2026.m)

This MATLAB script defines the machine parameters and runs the Simulink simulation to reproduce the results shown in Fig. 6 of the Actuator journal publication.

### [`BearinglessMotorSimulation.slx`](BearinglessMotorSimulation.slx)

Top level Simulink model that integrates both the universal model and the controller.

### [`Plant.slx`](Plant.slx)

The universal bearingless motor model (plant). This corresponds to Fig. 3 in both IMSB19 and Actuator papers.

### [`Controller.slx`](Controller.slx)

The universal force and torque controller. This corresponds to Fig. 4 in both IMSB19 and Actuator papers.

## How To Reproduce Simulation Results of ISMB19

To reproduce the simulation result of ISMB19:

1. Open [`setup_ismb_19.m`](./setup_ismb_19.m).
2. Specify the desired winding configuration (e.g., MCI) as follows:

```matlab
% Update winding_configuration to be 'Separate', 'MP', 'DNMP', 'Bridge',
% 'Parallel', or 'MCI' to indicate the type of winding to simulate (see 
% Fig. 2) 
winding_configuration = "MCI";
```

3. Run [`setup_ismb_19.m`](./setup_ismb_19.m).
  This will reproduce the Simulink simulation results presented in Fig. 6 of ISMB19, as follows:

| **MP**                                    | **Bridge DPNV**                               | **Parallel DPNV**                               | **MCI**                                    |
|:-----------------------------------------:|:---------------------------------------------:|:-----------------------------------------------:|:------------------------------------------:|
| <img src=images/mp-ismb-19.svg width=300> | <img src=images/bridge-ismb-19.svg width=300> | <img src=images/parallel-ismb-19.svg width=300> | <img src=images/mci-ismb-19.svg width=300> |

## How To Reproduce Simulation Results of Actuator

To reproduce the simulation result of Actuator:

1. Open [`setup_actuator_2026.m`](./setup_actuator_2026.m).
2. Specify the desired winding configuration (e.g., MCI) as follows:

```matlab
% Update winding_configuration to be 'Separate', 'MP', 'DNMP', 'Bridge',
% 'Parallel', or 'MCI' to indicate the type of winding to simulate (see 
% Fig. 2) 
winding_configuration = "MCI";
```

3. Run [`setup_actuator_2026.m`](./setup_actuator_2026.m).
  This will reproduce the Simulink simulation results presented in Fig. 6 of Actuator, as follows:

| **MP**                                    | **Bridge DPNV**                               | **Parallel DPNV**                               | **MCI**                                    |
|:-----------------------------------------:|:---------------------------------------------:|:-----------------------------------------------:|:------------------------------------------:|
| <img src=images/mp-actuator-2026.svg width=300> | <img src=images/bridge-actuator-2026.svg width=300> | <img src=images/parallel-actuator-2026.svg width=300> | <img src=images/mci-actuator-2026.svg width=300> |


*Note: the Simulink files were created using MATLAB R2024b.*
