# Model Routing Policy

The main agent uses Luna XHigh and should handle most work directly.

Do not delegate by default.

## Use Main directly for

- reading and searching code
- routine code edits
- YAML and launch changes
- parameter changes
- ROS topic/remap work
- normal compilation errors
- straightforward runtime debugging
- shell commands
- normal implementation work

## Use Debugger only when

- root cause remains unknown after initial inspection
- a reasonable previous fix failed
- runtime behavior contradicts apparent code logic
- the issue spans multiple subsystems
- planner/state-machine/algorithm behavior is unclear
- upstream algorithm assumptions may not fit the robot/environment

Debugger uses Sol High.

Its job is to determine:
1. earliest failure point
2. evidence
3. root-cause hypothesis
4. minimal repair plan

Debugger should not perform broad implementation.

## Use Implementer when

- a repair plan already exists
- several related files must be changed
- implementation is non-trivial

Implementer uses Luna XHigh.

## Use Verifier when

- build/runtime verification is needed
- ROS nodes/topics/TF/logs must be checked
- planner output must be validated

Verifier uses Luna XHigh.

Verifier should collect evidence and avoid redesigning the fix.

## Use Reviewer only when

- the patch is large
- multiple subsystems changed
- algorithm behavior changed
- regression risk is meaningful

Reviewer uses Terra High.

Do not run Reviewer for routine patches.

# Preferred workflows

Simple task:
Main

Normal bug:
Main -> Verify

Complex unknown bug:
Main -> Debugger -> Implementer -> Verifier

Large/high-risk fix:
Main -> Debugger -> Implementer -> Verifier -> Reviewer