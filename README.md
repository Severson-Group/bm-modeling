
# BM Modeling

## Introduction

This repository houses shared bearingless motor control models developed by the [eLev Lab](https://elev.umn.edu/) and its collaborators.

## Folder Configuration

### [`UniversalBM`](./UniversalBM/)

This simulation package includes all necessary files to simulate any bearingless motor with a combined winding. This model was developed in the following publication:

```markdown
Takahiro NOGUCHI, Mohamadhasan MOKHTARABADI, Kamisetti N V PRASAD, Wolfgang GRUBER and Eric L. SEVERSON,
"Model and Control Framework for Bearingless Motors with Combined Windings"
19th International Symposium on Magnetic Bearings (ISMB19), 2025.
```

## [`MP`](./MP/)

This folder contains Simulink files for multiphase combined windings as well as an example implementation of system identification techniques.

## [`PMSM`](./PMSM/)

This folder contains Simulink files for simulating current regulation of a standard three phase motor.

### [`prototypes`](./prototypes/)

This folder contains the Simulink model used to control bearingless motor prototypes in the [eLev Lab](https://elev.umn.edu/).
