# BM Modeling

## Introduction

This repository houses bearingless motor control models developed by the eLev lab and its collaborators, made freely available.

## Folder Configuration

### [`UniversalBM`](./UniversalBM/)
This simulation package includes all the necessary files to simulate the universal bearingless motor model. This model was developed based on the following publication:

```markdown
Takahiro NOGUCHI, Mohamadhasan MOKHTARABADI, Kamisetti N V PRASAD, Wolfgang GRUBER and Eric L. SEVERSON,
"Model and Control Framework for Bearingless Motors with Combined Windings"
19th International Symposium on Magnetic Bearings (ISMB19), 2025.
```

## [`MP`](./MP/)
This folder contains Simulink files for multiphase only, along with the implementation of a system identification.

## [`PMSM`](./PMSM/)
This folder contains Simulink files for simulating current regulation of a normal motor.

### [`prototypes`](./prototypes/)
This folder contains the Simulink model used to control bearingless motor prototypes in eLev lab.
