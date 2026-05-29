# concert_isaac

Isaac Sim / Isaac Lab integration of the CONCERT base. Loads the robot inside
a BIM environment converted from IFC/glb.

## Launch the simulation

From the Isaac Lab install dir:

```bash
./isaaclab.sh -p ~/<forest_ws>/src/concert_description/concert_isaac/scripts/simulation/simulate_bim.py --env palazzina-san-bartolomeo
```

### Available `--env`

The simulator looks for USD files under
`source/concert_isaac/assets/usd/environments/<env>/<env>.usd`. Currently:

- `cantamessa`
- `chiaraman`
- `nika`
- `palazzina-san-bartolomeo`


