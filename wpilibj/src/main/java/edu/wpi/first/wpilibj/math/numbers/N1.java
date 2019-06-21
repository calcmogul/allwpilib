/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N1 extends Num implements Nat<N1> {
    private N1() {
    }

    @Override
    public int getNum() {
        return 1;
    }

    public static final N1 N1 = new N1();
}
