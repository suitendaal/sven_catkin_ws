Alle files zijn opgenomen met een frequentie van 200Hz

demo1
-stiffness x en y naar 100
-rotational stiffness naar 5
-stiffness z naar 0
-scenario van bumpen in rubber tegel
-schrift erop gelegd voor glijden voor simultaneous impact

replay1.1
-stiffness z naar 6000
-2 extra rubberen tegels
-duidelijke simultaneous impact

replay1.2
-zelfde als replay1.1

replay1.3
-zelfde als replay1.1

replay1.4
-zelfde als replay1.1

replay1.5
-zelfde als replay1.1
-stiffness x en y naar 200

replay1.6
-zelfde als replay1.5

replay1.7
-zelfde als replay1.5

replay1.8
-zelfde als replay1.5

replay1.9
-zelfde als replay1.1
-rotational stiffness naar 10

replay1.10
-zelfde als replay1.9

replay1.11
-zelfde als replay1.9

replay1.12
-zelfde als replay1.9

replay1.13
-zelfde als replay1.1
-stiffness x en y naar 200
-rotational stiffness naar 10

replay1.14
-zelfde als replay1.13

replay1.15
-zelfde als replay1.13
-nullspace stiffnes naar 10 (per ongeluk)

replay1.16
-zelfde als replay1.13

replay1.17
-zelfde als replay1.13

demo2
-teleoperation demo met htc vive
-stiffness x, y, z naar 2000
-rotational stiffness naar 20

demo3
-zelfde als demo2
-mislukt: tau_J_range_violation

demo4
-zelfde als demo2

demo5
-zelfde als demo2

demo6
-zelfde als demo2
-niet al te stevige impact

demo7
-zelfde als demo2
-rotational stiffness naar 40
-stevige impact

demo8
-zelfde als demo7
-niet al te stevige impact
-niet helemaal duidelijk of er een dubbele impact plaatsvindt

demo9
-zelfde als demo7
-stevige impact

demo10
-zelfde als demo7

demo11
-zelfde als demo7
-stiffness z naar 6000
-mislukt: tau_J_range_violation

demo12
-zelfde als demo11
-mislukt: joint_velocity_violation

demo13
-zelfde als demo2
-stiffness x,y,z naar 400
-rotational stiffness naar 30

demo14
-zelfde als demo13
-niet duidelijk of er een dubbele impact was

demo15
-zelfde als demo13

demo16
-zelfde als demo13

demo17
-replay demo13
-stiffness hetzelfde

demo18
-zelfde als demo17
-impact control mode op 1: position only

demo19
-promps gemaakt met demo2, demo4 en demo5
-jump detector getest: doet het
-sven_ros_controller getest: orientation lijkt nog niet helemaal te kloppen
-stiffness x,y,z naar 400
-rotational stiffness naar 30
-tijdens impact wordt impact mode 1 gebruikt: position only

demo20
-zelfde als demo19

demo21
-zelfde als demo19
-stiffness x,y,z naar 1000
-er is zelfs een jump gedetecteerd na het switchen van phases

demo22
-zelfde als demo21
-zelfs 2 jumps gedetecteerd na switchen van phases

demo23
-zelfde als demo21
-ook 2 jumps na switchen

demo24
-zelfde als demo19
-impact mode is feedforward (2)

demo25
-zelfde als demo24

demo26
-zelfde als demo24
-stiffness x,y,z naar 1000
-1 jump detected na wisselen van phase

demo27
-zelfde als demo26
-ook een jump gedetecteerd


Filmpjes
1: replay1.1
2: replay1.2
3: replay1.3
4: replay1.4
5: replay1.5
6: replay1.6
7: replay1.7
8: replay1.8
9: replay1.9
10: replay1.10
11: replay1.11
12: replay1.12
13: replay1.13
14: replay1.14
15: replay1.15
16: replay1.16
17: replay1.17
18: mislukt
19: mislukt
20: mislukt
21: demo2
22: demo3
23: mislukt
24: demo4
25: demo5
26: demo6
27: demo7
28: demo8
29: demo9
30: demo10
31: demo11
32: demo12
33: demo13
33: mislukt
34: demo13
35: demo14
36: demo15
37: demo16
38: mislukt
39: demo17
40: demo18
41: demo19
42: demo20
43: demo21
44: demo22
45: demo23
46: demo24
47: demo25
48: demo26
49: demo27

