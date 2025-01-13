```bash

khaled@khaled:~/jordan_ws/src/petra_robot/urdf$ gz sdf -p petra_robot.urdf > petra_robot.sdf

khaled@khaled:~/jordan_ws/src/petra_robot$ mkdir models

khaled@khaled:~/jordan_ws/src/petra_robot$ mkdir worlds

khaled@khaled:~/jordan_ws/src/petra_robot/models$ mkdir petra_robot

khaled@khaled:~/jordan_ws/src/petra_robot/models$ cd petra_robot/

khaled@khaled:~/jordan_ws/src/petra_robot/models/petra_robot$ mkdir meshes

khaled@khaled:~/jordan_ws/src/petra_robot/models/petra_robot$ touch model.sdf model.config

models/
└── petra_robot/
    ├── meshes/
    ├── model.sdf
    └── model.config

<?xml version='1.0' encoding='utf-8'?>
<model>
  <name>yahboomcar</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>khaled gabr</name>
    <email>khaledgabr77@gmail.com</email>
  </author>
  <description>A model of Ackermann R2 car.</description>
</model>

```

```xml

<?xml version='1.0' encoding='utf-8'?>
<model>
  <name>petra_robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>khaled gabr</name>
    <email>khaledgabr77@gmail.com</email>
  </author>
  <description>A model of petra robot</description>
</model>

```