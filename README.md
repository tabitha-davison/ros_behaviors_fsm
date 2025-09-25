# Comprobo FSM warmup project
By Tabitha D, and Julian S
09-24-2025

## Sheriff bot

This project interfaces with a neato to transform it into the _sheriff_
bot. The sheriff both starts by walking in a ___very intimidating___
star pattern. If anyone gets too close, the sheriff will calmly finish
their star before snapping into action and chasing down the intruder
who dares to interfere. The sheriff will either stop before the intruder
and give them a good talking to or, should the intruder be deft enough to
escape, look around for the intruder.

Starts in star() state -> moves to chase() state if someone detected ->
move to lookout() state if perpetrator lost -> move to star() state after
lookout() completed.

## Running

This package is called comprobo_project_1. Please build using the
following command in your root ros directory after sourcing.
```
colcon build --symlink-install --packages-select comprobo_project_1
```

and then you can run the following after sourcing your local setup.bash.

```
ros2 run comprobo_project_1 FSM
```
