# End-Effector
## Specifics
- Mounted cams: Primary Realsense stereocamera, [secondary arducam](assets/Datasheet_camera.pdf)
- Mounted electronic boards: end effector board, motor driver board
- Lead of the screw: 2 mm
- Fingers closing time: 4 s at 416 rpm (motor speed)
- Number of spins to close the fingers: 56 spins of the motor axle
- Grip width: ~7 mm 
- Normal force on a <u>single</u> finger for 1kg of mass gripped ~24,5
- Input torque on the leadscrew of 1kg of mass gripped: 0,105 $Nm$
![](assets/Pasted image 20260527171842.png)
## Leadscrew specifics
- Diameter: 8mm
- Lead of the screw: 2 mm
- Double opposite thread
## Gears specifics
- Reduction: 1,6 $\frac{\omega_{drive}}{\omega_{driven}}$
- Module: 1
- Driven's teeth: 48
- Driven's Pitch diameter: 48 mm
- Drive's teeth: 30
- Drive's Pitch diameter 30mm
- Center distance: 39 mm
- Pressure angle: 20°
- Teeth thickness: 10 mm

## Motor specifics
Type: stepper nema 11  
![](assets/Pasted image 20260527171036.png)
[Link to stepper motor](https://www.omc-stepperonline.com/it/nema-11-bipolare-1-8deg-12ncm-17oz-in-0-67a-6-2v-28x28x51mm-4-fili-11hs20-0674s)
## Assembly procedure
### Items
- Scatolato  
![](assets/Pasted image 20260602161559.png)
- 2 8x11x7mm ball bearings (int d. x ext d. x thichness)
- Lead screw collar  
![](assets/Pasted image 20260602162628.png)
- Lead screw (the unit is in mm)  
![](assets/Pasted image 20260611134739.png)
- 2 lead screws nuts  
![](assets/Pasted image 20260602164321.png)
- 2 fingers attachments  
![](assets/Pasted image 20260602164424.png)
- Carbon tube: lenght 140 mm, ext d. 8 mm, int d. 6mm
- 2 Carbonfiber-bar-fixers  
![](assets/Pasted image 20260602171710.png)
- Incremental encoder with board  
![](assets/Pasted image 20260602173029.png)
- Incremental encoder support and lid  
![](assets/Pasted image 20260602173105.png)
- Driver support  
![](assets/Pasted image 20260602174009.png)
- Lid  
![](assets/Pasted image 20260609182917.png)
- Nema 11 motor  
![](assets/Pasted image 20260609182941.png)
- Camera attachment  
![](assets/Pasted image 20260602175607.png)
- 1 End stop
- Drive wheel  
![](assets/Pasted image 20260611132748.png)
- Driven wheel  
![](assets/Pasted image 20260611132817.png)
- Linear brush bearings 8x16x15 (int. d. x ext. d. x thickness)
- Flexible camera base  
![](assets/Pasted image 20260611133057.png)
- Flexible camera support  
![](assets/Pasted image 20260611133244.png)
- [Flexible arm](https://it.rs-online.com/web/p/sistemi-di-raffreddamento-macchine-utensili/6235094)  
![](assets/Pasted image 20260611133311.png)
- End effector electronic board
- Electronic board lid and slider  
![](assets/Pasted image 20260611133413.png)

### Bolts
(m a x y --> a=diameter, y=lenght)

- 14 m3 inserts (ext d. 4mm, lenght 4 mm)
- 9 m2x8
- 8 m2x12
- 4 m2x16
- m2x20
- 13 m2 nuts
- 4 m3x8
- 6 m2x12 
- 12 m3x16
- 5 m3x20
- m3x30
- m3x35
- 12 m3 nuts
- 2 m4x10 
- 2 m4 nuts

## Assembly process 
1. Put in 8 m3 inserts in the scatolato where shown (axis in fingure)  
![](assets/Pasted image 20260602173700.png)
2. Put the 2 8x11x7 ball bearings by pushing them in the housings.   
![](assets/Pasted image 20260602162331.png)
3. Put the collar on the short end of the leadscrew, fix it with a m2x12 bolt and nut. (the arrow direction shows how to insert the bolt)  
![](assets/Pasted image 20260602163133.png)
4. Put in 2 m3 inserts for falange attachment.  
![](assets/Pasted image 20260602170831.png|500)
5. Insert the leadscrew, making it pass through the nnuts and the finger attachments, being careful to follow the order of the image.  
![](assets/Pasted image 20260602164112.png)
6. Fix the driven wheel with a m2x20 bolt.  
![](assets/Pasted image 20260602164743.png)
7. Put the nuts in the finger attachment housings (don't fix them yet!), then lead them to the end stop, so that they are aligned. Use something to keep the nut's holes aligned with the attachment's ones (i suggest using a wrench key through the holes as shown in the second image).  
![](assets/Pasted image 20260602164944.png)  
![](assets/Pasted image 20260602165227.png)
8. Bring the attachments to the other end stop, then fix them wit 4 m3x16 bolts and nut for each.  
![](assets/Pasted image 20260602165508.png)
9. Insert 2 brush bearing on the carbon fiber tube, then put them in their finger attachment housings, lastly fix the lower attachment parts with 2 m3x20 bolts each.  
![](assets/Pasted image 20260602170956.png)
10. Fix the carbon fiver tube with the carbon-fiber-fixers using 2 m2x16 bolts and laterally inserted nuts each.  
![](assets/Pasted image 20260602171449.png)
11. Fix the incremental encoder to his support and lid with a m2x12 bolt.  
![](assets/Pasted image 20260602172903.png)
12. Fix the encoder support to the scatolato wit 3 m3x12 bolts.  
![](assets/Pasted image 20260602173418.png)
13. Fix the nema 11 stepper motor to the lid with 4 m2x8 bolts.  
![](assets/Pasted image 20260602174334.png)
14. Fix the drive wheel to the motor with a m2x8 bolt.  
![](assets/Pasted image 20260602174600.png)
14. Put 2 m3 inserts in the driver support.  
![](assets/Pasted image 20260602173853.png)
15. Fix the driver and its lid to its support with 4 m2x8  
![](assets/Pasted image 20260611165744.png)
16. Fix the support to the lid with 2 m3x12 bolts. (multiple prospective in the images)  
![](assets/Pasted image 20260602175019.png)  
![](assets/Pasted image 20260602175030.png)
17. Fix the camera support with 2 m3x8 bolts and nuts. (multiple prospective in the images)  
![](assets/Pasted image 20260602175414.png)  
![](assets/Pasted image 20260602175442.png)
18. Fix the end-stop with 2 m2x12 bolts and nuts.  
![](assets/Pasted image 20260602180714.png)
19. Fix the flexible arm to the base of the flexible camera with 2 m4x10 bolts and nuts. If the arm is lose, use some scotch to make it sturdy.  
![](assets/Pasted image 20260602180958.png)  
![](assets/Pasted image 20260602181140.png)
20. Mount the flexible camera support to the arm with a m3x30 nut and bolt (if the hole isn't present, drill it), then fix the camera with 4 m2x12 screws and bolts.  
![](assets/Pasted image 20260602181606.png)
21. Fix the lid to the scatolato wit 4 bolts, which lengths are listed form the RIGHT one to the left one IN THE IMAGE: m3x16 <u>plus m3 nut</u> laterally inserted, m3x20, m3x10, m3x8.  
![](assets/Pasted image 20260602182502.png)
22. Fix the electronic board with his lid using 3 m2x16 bolts.  
![](assets/Pasted image 20260602183040.png)
23. Insert the slider, and fix it with a m3x12 bolt and nut.  
![](assets/Pasted image 20260602183254.png)
24. Attach the end effector the wrist with 2 m3x35 bolts and nuts.   
![](assets/Pasted image 20260602183346.png)

## Fingers assembly procedure
- mount the finger on the finger attachment.  
![](assets/Pasted image 20260607165504.png)
- Use a m4x20 plus nut to fix it.  
![](assets/Pasted image 20260607165634.png)

## Sampling fingers
### Items
- Right sampling finger  
![](assets/Pasted image 20260609172000.png)
- Left sampling finger  
![](assets/Pasted image 20260609172026.png)
- 2 m2 inserts (ext diameter <u>al least</u> 4mm)
- 2 sampling finger covers
- 2 m2x10 bolts
- 2 pressure-distribution disk  
![](assets/Pasted image 20260609172109.png)
- 2 pressure sensors  
![](assets/Pasted image 20260609172146.png)
- Right sensor cover  
![](assets/Pasted image 20260609172403.png)  
![](assets/Pasted image 20260609172420.png)
- Left sensor cover  
![](assets/Pasted image 20260609172442.png)  
![](assets/Pasted image 20260609172455.png|400)
### Assembly procedure
1. Put in a m2 insert  
![](assets/Pasted image 20260607170435.png)
2. Put on the tpu cover and fix it wit a m2x10 bolt.  
![](assets/Pasted image 20260607170312.png)  
![](assets/Pasted image 20260607170209.png)
3. Put the pressure-distribution disk on the pressure sensor, then put them inside the finger, trying to put the center of the circular part of the sensor as close to the centerline of the finger as possible.  
![](assets/Pasted image 20260607170706.png)
4. Put on the TPU sensor cover.  
![](assets/Pasted image 20260607170807.png)  
![](assets/Pasted image 20260607170817.png)

## Manipulator fingers
### Items needed
-  Left manipulation finger  
![](assets/Pasted image 20260609172728.png)
- right manipulation finger  
![](assets/Pasted image 20260609172827.png)
- 2 sensor pre-loaders  
![](assets/Pasted image 20260609172854.png)
- Left TPU cover  
![](assets/Pasted image 20260609172945.png)
- Right TPU cover  
![](assets/Pasted image 20260609173019.png)
- Left TPU sensor cover  
![](assets/Pasted image 20260609173146.png)
- Right TPU sensor cover  
![](assets/Pasted image 20260609173120.png|173)
- 4 pressure sensors  
![](assets/Pasted image 20260609172146.png)
- 2 pressure-distribution disk  
![](assets/Pasted image 20260609172109.png)  
- 4 m2 inserts (external diameter <u>at least</u> 4 mm)
- 2 m2x10 bolts
- 2m2x12 bolts

### Assembly process
1. Put in 2 m2 inserts (2 images for different points of view)  
![](assets/Pasted image 20260607171202.png)  
![](assets/Pasted image 20260607171211.png)
2. Put on the TPU cover and fix it in place with a m2x10 bolt.  
![](assets/Pasted image 20260607171336.png)
3. Put in place the sensor pre-loader, and fix it in place with a m2x12  
![](assets/Pasted image 20260607171442.png)  
![](assets/Pasted image 20260607171514.png)
4. Put in place the 2 pressure sensors, being careful to have them as centered as possible in their housings.  
![](assets/Pasted image 20260607171643.png)
5. Put the pressure-distributor disk in place, then use the pre-loader to fix in place both the sensor and the disk.  
![](assets/Pasted image 20260607171738.png)
6. Put the sensor TPU sensor cover in place.  
![](assets/Pasted image 20260607171918.png)

## Syringe
The two syringe fingers are <u>different from each other</u>!  
To fix the syringe, put the lips between the finger components and then fix them together with 2 m2x10 bolts. Do it also for the other finger.  
![](assets/Pasted image 20260609165001.png) 

## Phometer
Use 2 m3x16 bolts to fix the PH-meter.  
![](assets/Pasted image 20260607172406.png)  

## Calculations
In these calculations it was assumed that the touching surfaces were both made of steel.  
Considered lifted mass: M=1 kg  
static friction factor: $\mu=0,2$  
dynamic friction factor of the leadscrew: $f=0,44$  
lead of the screw: 2mm  
screw diameter: 8  
$g=9,8 \frac{m}{s^2 }$  
![](assets/Pasted image 20260611143151.png)  
$P=M*g$  
$P=2T$  
$F=\mu N$  
$\begin{cases}
P=M*g \\
P=2T \\
T=\mu F
\end{cases} \Rightarrow F=\frac{M*g}{2*\mu}=24,5N$  

Normal force on single finger: 24,5 N  
Lead angle of threads:  
$\alpha=\arctan\left(\frac{P}{\pi*d}\right)$  

Lead screw efficiency (1):  
![](assets/Pasted image 20260611141741.png)  
Necessary input torque (1):  
![](assets/Pasted image 20260611143003.png)  
Input torque on the leadscrew of 1kg of mass gripped: $C=0,105 Nm$  

## References
(1) [https://contigroup.it/Catalogo/Catalogo-EN.pdf](https://contigroup.it/Catalogo/Catalogo-EN.pdf)  
(2) [https://en.wikipedia.org/wiki/Helix_angle](https://en.wikipedia.org/wiki/Helix_angle) (here you can find the complementary angle of the one indicated before)