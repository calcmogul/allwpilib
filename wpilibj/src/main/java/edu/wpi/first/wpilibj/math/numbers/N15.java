/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N15 extends Num implements Nat<N15> {
    private N15() {
    }

    @Override
    public int getNum() {
        return 15;
    }

    public static final N15 N15 = new N15();
}
