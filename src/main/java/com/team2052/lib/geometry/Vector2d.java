package com.team2052.lib.geometry;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public class Vector2d {
  private Translation2d vector;

  public Vector2d(Translation2d vector) {
    this.vector = vector;
  }

  public Vector2d(double x, double y) {
    this.vector = new Translation2d(x, y);
  }

  public Vector2d(Angle theta, double magnitude) {
    this.vector = new Translation2d(magnitude, new Rotation2d(theta.in(Radians)));
  }

  public Translation2d getVectorAsTranslation() {
    return vector;
  }

  public double getX() {
    return vector.getX();
  }

  public double getY() {
    return vector.getY();
  }

  public Vector2d plus(Vector2d other) {
    return new Vector2d(this.vector.plus(other.getVectorAsTranslation()));
  }

  public Vector2d minus(Vector2d other) {
    return new Vector2d(this.vector.minus(other.getVectorAsTranslation()));
  }

  public double getMagnitude() {
    return vector.getNorm();
  }

  public Angle getDirection() {
    return Radians.of(vector.getAngle().getRadians());
  }

  public double getDotProduct(Vector2d other) {
    return this.getX() * other.getX() + this.getY() * other.getY();
  }

  public double getCrossProduct(Vector2d other) {
    return this.getX() * other.getY() - this.getY() * other.getX();
  }

  public void scale(double scalar) {
    vector.times(scalar);
  }

  public Vector2d scaled(double scalar) {
    return new Vector2d(vector.times(scalar));
  }

  public void add(Vector2d other) {
    vector = vector.plus(other.getVectorAsTranslation());
  }

  public void subtract(Vector2d other) {
    vector = vector.minus(other.getVectorAsTranslation());
  }

  public void invert() {
    vector = vector.unaryMinus();
  }

  public Translation2d applyVector(Translation2d point) {
    return point.plus(vector);
  }

  public Vector2d rotated(Rotation2d unaryMinus) {
    return new Vector2d(vector.rotateBy(unaryMinus));
  }
}
