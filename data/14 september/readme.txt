Vandaag staat in het teken van demonstraties doen met de HTC Vive controller.
De eerste demonstraties zijn de gebruikelijke taak: de robot bumpt in een stapel van rubberen tegels en komt weer omhoog.
Daarna worden er demonstraties gedaan waarbij de robot een doos een eindje moet duwen. Dit gebeurt op zo'n manier dat de robot aankomt met een impact. Hierbij zijn een vaste start en eindlocatie vastgestelt met tape op de tafel.
Dit laatste niet aan toegekomen. Wel aan het testen van verschillende "control modes" tijdens de impact phase.

demo1
-stiffness x,y,z naar 1000
-rotational stiffnes 30
-niet echt een impact

demo2
-zelfde als demo1
-betere impact, bleef wel lang hangen in tegels

demo3
-zelfde als demo1
-zelfde soort impact als demo2

demo4
-zelfde als demo1
-mislukt: te hoge joint torques

demo5
-zelfde als demo1
-niet hele goede impact, heel kort. Niet duidelijk of het een dubbele impact is

demo6
-zelfde als demo1
-prima impact

demo7
-zelfde als demo1
-prima impact

demo8
-zelfde als demo1
-mislukt: te hoge joint torques

demo9
-zelfde als demo1
-prima impact

demo10
-zelfde als demo1
-prima impact

demo11
-zelfde als demo1
-schrift bovenop rubber tegels gelegd voor gladheid
-prima impact

demo12
-zelfde als demo11
-prima impact

demo13
-zelfde als demo11
-dikke impact

demo14
-zelfde als demo11
-lichte impact, volgens mij geen dubbele

demo15
-zelfde als demo11
-prima impact

demo16
-zelfde als demo11
-prima impact

demo17
-zelfde als demo11
-prima impact

demo18
-zelfde als demo11
-prima impact
-schrift leek verplaatst voor demo, dus goed gelegd

demo19
-zelfde als demo11
-prima impact

demo20
-zelfde als demo11
-prima impact

exec1
-offline algoritme uitgevoerd met replay4.1, replay4.2, replay4.3 van 23 juli
-het resultaat van de gecreerde promps met jump detector
-PositionFeedback tijdens impact phase
-stiffness in x,y,z naar 2000, 2000, 6000
-rotational stiffness naar 30
-langzame beweging, 1 impact gedetecteerd

exec2
-zelfde als exec1, maar nu met schrift op rubberen tegels

exec3
-zelfde als exec2, maar nu met FeedForward aks control mode tijdens impact
-in het begin stond de vorige sven ros controller nog aan, dus begin van data moet worden weggehaald

exec4
-zelfde als exec2
-offline algoritme uitgevoerd met demo2, demo4, demo5 van 6 september
-stiffness in x,y,z naar 1000
-impact_interval_threshold van 0.1s naar 0.2s gezet

exec5
-zelfde als exec4
-FeedForward mode tijdens impact phase



videos
1: demo1
2: demo2
3: demo3
4: demo4
5: demo5
6: demo6
7: demo7
8: demo8
9: demo9
10: demo10
11: demo11
12: demo12
13: demo13
14: demo14
15: demo15
16: demo16
17: demo17
18: demo18
19: demo19
20: demo20
21: exec1
22: exec2
23: exec3
24: exec4
25: exec5

