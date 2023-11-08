# Notes on `best_deviation_angle()` Function:
This function return the best heading deviation angle to avoid an obstacle.

It uses the sonic scanner to scan the environment. The deviation angle is the centre scan angle of a contineous range of scan angles with the required free range.
 
## Problem:
To avoid an obstacle, a gap that is wide enough for the vehicle to pass through is required. The best deviation angle would then be in the middle of the gap.

The problem is that a single scan in a direction is very narrow. Each sonic scan is a 7 degree cone.
Therefore multiple scans in a contiguous range is required. 
The contiguous scan range must cover a width equal to the vehicle's width

How many scans would provide the required range?
The width of the cone is distance dependent. E.g. @ 10cm (adjacent and hypotenuse) the width (opposite for 7 degree angle) would be about 1.22cm.

A 70 degree scan at a distance of 15 cm would give a width of 17.2cm, that is 10 continuous scan points.

## Also:
What if no solution is found?

Turn 90 degrees and scan again?

## Scan Angle:
The scan angle is the position of the scanner on the scan arc created by servo motor
The scanner is pointing -
* straight ahead when the scan angle = 90 degrees
* to the right when the scan-angle < 90 degrees
* to the left when the scan-angle > 90 degrees

The actual arc of the servo is not a full 180 degrees.
What range and what scan step should be used?

## Scan Step:
A scan step of 10 degrees will result in gaps in the scan arc.

* At 10 degrees the scan would cover from 10 - (7 / 2) = 6.5 degrees to 10 + (7 / 2) = 13.5 degrees
* At 20 degrees the scan would cover from 20 - (7 / 2) = 16.5 degrees to 20 + (7 / 2) = 23.5 degrees

Starting at 10 degrees a 7 degree step would not result in gaps - 
* At 10 degrees the scan would cover from 10 - (7 / 2) = 6.5 degrees to 10 + (7 / 2) = 13.5 degrees
* At 17 degrees the scan would cover from 17 - (7 / 2) = 13.5 degrees to 17 + (7 / 2) = 20.5 degrees
* At 24 degrees the scan would cover from 24 - (7 / 2) = 20.5 degrees to 17 + (7 / 2) = 27.5 degrees

## Scan range:
90 degrees is directly ahead with a scan area from 86.5 to 93.5 degrees (3.5 degrees on each side).

Using the 7 degree step from above the next scan point would be 83 degrees.
A scan direction of 83 degree would cover 86.5 to 79.5 degrees.

The left and right side must have an equal number of scan points.
Scan points per side would be 90 / 7 which gives a whole number of 12.

Therefore the scan start point would be 90 - (7 x 12) = 90 - 84 = 6 degrees and the scan end point would be 90 + 84 = 174 degrees.
This would give a total scan arc of 6 - 3.5 = 2.5 and 174 + 3.5 = 177.5

The scan points would be 6, 13, 20, 27, 34, 41, 48, 55, 62, 69, 76, 83, 90, 97, 104, 111, 118, 125, 132, 139, 146, 153, 160, 167 and 174
* Front scan point: 90
* Left of front:  97, 104, 111, 118, 125, 132, 139, 146, 153, 160, 167, 174
* Right of front: 83,  76,  69,  62,  55,  48,  41,  34,  27,  20,  13,   6

## The deviation angle:
The deviation angle is the angle between the center line (90 degrees) and the scanning direction on the scan arc. 

It gives the degrees the vehicle must turn to align its center line with the scan direction. 

The deviation angle = 90 - scan angle
* If positive turn right
* If negative turn left
