# LevionArm

This repository contains required packages to control "Levion Arm", which is equipped on the Floating Platform for Zero-G lab at University of Luxembourg.

## Installation

Clone the repo including submodules.

```bash
git clone --recurse-submodules -j8 git@github.com:aky-u/LevionArm.git
```

> [!NOTE]
> If you have cloned without submodules, use the following command to clone submodules.
>
> `git submodule update --init --recursive`

Install required packages.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## AK Series

[Cubemars support](https://www.cubemars.com/article.php?id=261)

- [AK80-8](https://www.cubemars.com/goods-1151-AK80-8.html)

## Related works

- <https://github.com/neurobionics/TMotorCANControl>

- <https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can>

- <https://github.com/SherbyRobotics/tmotor_ros>

- <https://github.com/OpenFieldAutomation-OFA/cubemars_hardware/tree/main>

## Trouble shooting

- [If Motor Failed Entering Both Modes](https://www.cubemars.com/article-330-If+Motor+Failed+Entering+Both+Modes.html)

<!-- > [!WARNING]
> -->
