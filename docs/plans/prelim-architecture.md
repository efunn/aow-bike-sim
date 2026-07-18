# Preliminary Architecture
- MuJoCo physics model of an active omni wheel (AOW) RC bike
- model the entire omni-wheel accurately for now; later, we can try approximations to simplify the simulation
- goal is to first balance the bike and perform basic maneuvers; more advanced maneuvers (divided into agility (e.g. doing drift turns) and gameplay (e.g. hitting a ball)) are the next target

## Physical model
- Active omni wheel based on [Spin Master Ducati Upriser](https://gizmodo.com/this-rc-ducati-motorcycle-is-packing-some-surprisingly-1836390814) and its Chinese clone [Hui Can HC-802](https://www.snhobbies.com/product_info.php?products_id=15254) rear wheel design.
- Physical specimen is scavenged from an HC-802: from video reviews, it appears identical in construction to the Ducati Upriser (likely uses same injection molds); overall, the main circuit board and body shell appear different, but internal components/drive/steering appear identical between Chinese clones and the original Spin Master toy
- I can take apart and measure all components in the omni wheel
- other than the omni wheel, the overall bike will have different geometry (weight/wheelbase/steering angle) than the Upriser/HC-802

## Sensors and servos
- for now, the physical bike will operate on an umbilical using ROBOTIS dynamixel servos (datasheets available in `docs/robotis/*`):
  - steering: XC330-T181 servo through a gear (with a possible ratio, or 1:1)
  - AOW driving: differential control by two XC430-W150 servos, through a belt drive (probably 3:1 ratio to give faster top speed, but adjustable)
- in addition to motor sensors, an AHRS will be equipped for measuring orientation: [TransducerM AHRS 9-Axis IMU for Robotics & Autonomous Vehicles (TM151)](https://www.syd-dynamics.com/transducerm_tm151-tm171/)
- 12V/5A power brick used for motors
- eventually, a linux microcontroller and 3s/11.1V battery will be used 

## Converting to MuJoCo model: 2 options

- (1) I measure and model every component of the omni wheel (gears/rollers/etc) and provide you the full model of the bike from OnShape (e.g. in URDF format, which OnShape can export) to generate in MuJoCo
- (2) I describe the omni wheel in text in great detail, and you generate the MuJoCo model based on this text description
- (3-N) ??? any options you come up with (most important thing is accuracy, I will do whatever work necessary to ensure this on the measurement/3D modelling side)
  - a list of all parameters needed to input to the model would be helpful!

## MuJoCo design considerations 
- the AOW rollers can be modelled as a truncated cone geom
- which contact mode to use? (`condim=6` for rolling friction? But how does this work for omni wheels...?)
- probably use elliptical friction cones and standard/default solvers
- any other important ones I missed?

## Performance Goals

### Baseline
- balance while standing still (minimize rear wheel/positional drift)
- pivot as fast as possible in either direction, stop, and stay upright
- drive forward and backward in a straight line (accelerate and decelerate as quickly as possible)
- drive forward and backward in circles (possible goals: tightest circle, fastest circle, most recoverable (can stop quickly at any position))
- etc. (just some ideas so far)

### Advanced (agility)
- starting from a given velocity, turn the bike 180 degrees as quickly as possible
- starting from stand still, turn the bike 180 degrees in the smallest area possible
- etc. (just some ideas so far)

### Advanced (gameplay)
- from a given distance and starting velocity, hit a ball as quickly as possible
- from a given distance and starting velocity, hit a ball as far as possible
- etc. (just some ideas so far)
