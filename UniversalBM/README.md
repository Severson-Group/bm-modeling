# Universal Bearingless Motor Model

## Introduction

This simulation package includes all the necessary files to simulate the universal bearingless motor model. This model was developed based on the following publication:

```markdown
Takahiro NOGUCHI, Mohamadhasan MOKHTARABADI, Kamisetti N V PRASAD, Wolfgang GRUBER and Eric L. SEVERSON,
"Model and Control Framework for Bearingless Motors with Combined Windings"
19th International Symposium on Magnetic Bearings (ISMB19), 2025.
```

### [`setup.m`](setup.m)
Main script to define machine parameters and run the Simulink simulation.

### [`BearinglessMotorSimulation.slx`](BearinglessMotorSimulation.slx)
Top level Simulink model that integrates both the universal model and the controller.

### [`Plant.slx`](Plant.slx)
The universal bearingless motor model (plant), corresponds to Fig. 3 in the paper.

### [`Controller.slx`](Controller.slx)
The universal force and torque controller, corresponds to Fig. 4 in the paper.

## Simulation

To run the simulation:

1. Open [`setup.m`](setup.m).
2. Specify the desired winding configuration (e.g., MCI) as follows:
   https://github.com/Severson-Group/bm-modeling/blob/ed2de350966ff855ba44755b870dd4060cbec2dd/UniversalBM/setup.m#L24-L26
3. Run [`setup.m`](setup.m). 
  This will reproduce the Simulink simulation results presented in Fig. 6, as follows:

| **Multiphase** | **Bridge DPNV** | **Parallel DPNV** | **MCI** |
|:-:|:-:|:-:|:-:|
| <img src=images/mp.svg width=300> | <img src=images/bridge.svg width=300> | <img src=images/parallel.svg width=300> | <img src=images/mci.svg width=300> |

*Note: the Simulink files were created using MATLAB R2024b.*
