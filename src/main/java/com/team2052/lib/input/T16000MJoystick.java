package com.team2052.lib.input;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class T16000MJoystick extends CommandJoystick {
  public T16000MJoystick(int port) {
    super(port);
  }

  // --- Stick Buttons ---
  public Trigger frontTrigger() {
    return button(1);
  }

  public Trigger middleThumbButton() {
    return button(2);
  }

  public Trigger leftThumbButton() {
    return button(3);
  }

  public Trigger rightThumbButton() {
    return button(4);
  }

  // --- Base Buttons (Left Side) ---
  public Trigger leftBaseTopLeft() {
    return button(11);
  }

  public Trigger leftBaseTopMiddle() {
    return button(12);
  }

  public Trigger leftBaseTopRight() {
    return button(13);
  }

  public Trigger leftBaseBottomLeft() {
    return button(16);
  }

  public Trigger leftBaseBottomMiddle() {
    return button(15);
  }

  public Trigger leftBaseBottomRight() {
    return button(14);
  }

  // --- Base Buttons (Right Side) ---
  public Trigger rightBaseTopLeft() {
    return button(7);
  }

  public Trigger rightBaseTopMiddle() {
    return button(6);
  }

  public Trigger rightBaseTopRight() {
    return button(5);
  }

  public Trigger rightBaseBottomLeft() {
    return button(8);
  }

  public Trigger rightBaseBottomMiddle() {
    return button(9);
  }

  public Trigger rightBaseBottomRight() {
    return button(10);
  }

  // --- Axes ---
  public double getX() {
    return getRawAxis(0);
  }

  public double getY() {
    return getRawAxis(1);
  }

  public double getTwist() {
    return getRawAxis(2);
  }

  public double getSlider() {
    return getRawAxis(3);
  }
}
