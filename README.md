## A* Path Planning algorithm in 3D

Finds path from start to goal using a L2 norm as heuristic.

Able to create your own world as a json package. Json must contain key words:
 - bounds
 - blocks
 - start
 - goal
 - margin
 - resolution

Usage:
```
python3 main.py -f <filename>
```
Example:
```
python3 main.py -f forest.json
```
Note that the filename must exist within the `worlds` directory