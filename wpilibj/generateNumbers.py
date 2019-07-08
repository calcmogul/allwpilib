#!/usr/bin/env python3
"""
Generates the numeric classes used as generics for the state space implementation
Will output `MAX_NUM + 1` files representing the numbers from 0 to `MAX_NUM`
"""

MAX_NUM = 20

with open("src/generate/GenericNumber.java.in", 'r') as templateFile:
    template = templateFile.read()
    for i in range(MAX_NUM + 1):
        with open(f"src/main/java/edu/wpi/first/wpilibj/math/numbers/N{i}.java",
                  "w") as f:
            f.write(template.format(i))
