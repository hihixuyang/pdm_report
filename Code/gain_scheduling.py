if error in tolerance:
    if state is coarse controller:
        change gains to fine controller
        reset integral term
        state is finecontroller
else:
    if state is fine controller:
        change gains to coarse controller
        reset integral term
        state is coarse controller
