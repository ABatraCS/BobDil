within BobDil.Experiments;

model DilSmokePlant
  "Minimal Modelica plant exposing exactly the VehicleFMI boundary"

  // Why this model exists.
  //
  // It is a *fixture*, not a vehicle. It presents the same three inputs and the
  // same output names as BobLib.Experiments.Standards.VehicleFMI, so the whole
  // FMI path -- omc export, unpacking, value-reference resolution, the kernel's
  // Model Exchange loader, and the fixed-step integrator -- can be exercised in
  // seconds instead of in however long a MultiBody full-vehicle build takes.
  //
  // It deliberately depends on no Modelica library at all, so it builds with a
  // bare omc and gives a signal even on a machine that cannot build BobLib.
  //
  // It is also the reference for how a real-time-friendly BobLib variant should
  // be written (architecture.md 1.5): every discontinuity here is a smooth
  // regularisation rather than a state event, and every division by velocity
  // carries a floor, because a state event costs an unbounded iteration inside
  // fmi2NewDiscreteStates and a DIL session always begins at rest.

  // --- driver inputs: identical names to VehicleFMI --------------------------
  input Real steeringAngleCommand(unit = "rad") "Handwheel angle, positive left";
  input Real acceleratorPedalCommand(unit = "1") "Accelerator, 0 to 1";
  input Real brakePedalCommand(unit = "1") "Brake, 0 to 1";

  // --- outputs: identical names to VehicleFMI --------------------------------
  output Real vehicleSpeed(unit = "m/s");
  output Real accX(unit = "m/s2");
  output Real accY(unit = "m/s2");
  output Real handwheelAngle(unit = "rad");
  output Real steerExcess(unit = "rad");
  output Real handwheelTorque(unit = "N.m") "Reaction torque; negate for the torque applied to the driver";
  output Real Fz_FL(unit = "N");
  output Real Fz_FR(unit = "N");
  output Real Fz_RL(unit = "N");
  output Real Fz_RR(unit = "N");
  output Real leftSteerAngle(unit = "rad");
  output Real rightSteerAngle(unit = "rad");
  output Real roll(unit = "rad");
  output Real sideslip(unit = "rad");
  output Real velX(unit = "m/s");
  output Real velY(unit = "m/s");
  output Real yawVel(unit = "rad/s");

  // --- parameters -------------------------------------------------------------
  parameter Real m = 280 "Total mass [kg]";
  parameter Real Izz = 105 "Yaw inertia [kg.m2]";
  parameter Real Ixx = 35 "Roll inertia [kg.m2]";
  parameter Real a = 0.78 "CG to front axle [m]";
  parameter Real b = 0.77 "CG to rear axle [m]";
  parameter Real trackF = 1.22 "Front track [m]";
  parameter Real trackR = 1.18 "Rear track [m]";
  parameter Real hCg = 0.30 "CG height [m]";
  parameter Real steerRatio = 4.0 "Handwheel angle per road wheel angle";
  parameter Real rackLimit = 2.2 "Handwheel travel limit [rad]";
  parameter Real Cf = 42000 "Front cornering stiffness [N/rad]";
  parameter Real Cr = 46000 "Rear cornering stiffness [N/rad]";
  parameter Real muFy = 1.55 "Peak lateral friction";
  parameter Real driveForce = 3400 "Peak longitudinal drive force [N]";
  parameter Real brakeForce = 5200 "Peak longitudinal brake force [N]";
  parameter Real dragArea = 1.15 "CdA [m2]";
  parameter Real liftArea = 2.85 "ClA [m2]";
  parameter Real rho = 1.19 "Air density [kg/m3]";
  parameter Real aeroBalance = 0.46 "Fraction of downforce at the front";
  parameter Real rollStiffness = 62000 "Total roll stiffness [N.m/rad]";
  parameter Real rollDamping = 3600 "Roll damping [N.m.s/rad]";
  parameter Real rollBalance = 0.55 "Fraction of roll stiffness at the front";
  parameter Real trail = 0.048 "Pneumatic plus mechanical trail [m]";
  parameter Real g = 9.80665;
  parameter Real vFloor = 1.0
    "Speed floor in every slip denominator [m/s]. Load-bearing: a session starts at rest.";

  // --- states -------------------------------------------------------------------
  Real vx(start = 0.0, fixed = true, unit = "m/s") "Body longitudinal velocity";
  Real vy(start = 0.0, fixed = true, unit = "m/s") "Body lateral velocity";
  Real yawRate(start = 0.0, fixed = true, unit = "rad/s");
  Real rollAngle(start = 0.0, fixed = true, unit = "rad");
  Real rollRate(start = 0.0, fixed = true, unit = "rad/s");

protected
  Real delta "Road wheel angle [rad]";
  Real vRef "Regularised forward speed, never zero";
  Real alphaF "Front slip angle [rad]";
  Real alphaR "Rear slip angle [rad]";
  Real FyF "Front axle lateral force [N]";
  Real FyR "Rear axle lateral force [N]";
  Real Fx "Net longitudinal force [N]";
  Real downforce "Total downforce [N]";
  Real frontLoad "Front axle vertical load [N]";
  Real rearLoad "Rear axle vertical load [N]";
  Real lateralTransferF "Front lateral load transfer [N]";
  Real lateralTransferR "Rear lateral load transfer [N]";
  Real longitudinalTransfer "Longitudinal load transfer [N]";

equation
  // Steering, with the rack limit reported rather than silently swallowed.
  handwheelAngle = min(rackLimit, max(-rackLimit, steeringAngleCommand));
  steerExcess = steeringAngleCommand - handwheelAngle;
  delta = handwheelAngle / steerRatio;
  leftSteerAngle = delta;
  rightSteerAngle = delta;

  // Smooth speed floor. sqrt(vx^2 + vFloor^2) is continuous and differentiable
  // everywhere, unlike max(abs(vx), vFloor), which would generate a state event
  // at exactly the operating point every session starts from.
  vRef = sqrt(vx * vx + vFloor * vFloor);

  // Slip angles, and lateral force saturated with tanh rather than clipped.
  // tanh keeps the model smooth at the limit, so the solver never has to locate
  // an event there.
  alphaF = delta - (vy + a * yawRate) / vRef;
  alphaR = -(vy - b * yawRate) / vRef;
  FyF = muFy * frontLoad * tanh(Cf * alphaF / (muFy * frontLoad));
  FyR = muFy * rearLoad * tanh(Cr * alphaR / (muFy * rearLoad));

  // Longitudinal: drive minus brake minus drag. The brake opposes motion
  // through tanh so there is no sign() discontinuity at standstill.
  Fx = driveForce * acceleratorPedalCommand
     - brakeForce * brakePedalCommand * tanh(vx / 0.5)
     - 0.5 * rho * dragArea * vx * vx * tanh(vx / 0.5);

  // Aerodynamics and load transfer.
  downforce = 0.5 * rho * liftArea * vx * vx;
  longitudinalTransfer = m * accX * hCg / (a + b);
  frontLoad = m * g * b / (a + b) + aeroBalance * downforce - longitudinalTransfer;
  rearLoad = m * g * a / (a + b) + (1 - aeroBalance) * downforce + longitudinalTransfer;
  lateralTransferF = rollBalance * m * accY * hCg / trackF;
  lateralTransferR = (1 - rollBalance) * m * accY * hCg / trackR;

  // +y is left, so a positive lateral acceleration loads the right-hand corners.
  Fz_FL = max(0, 0.5 * frontLoad - lateralTransferF);
  Fz_FR = max(0, 0.5 * frontLoad + lateralTransferF);
  Fz_RL = max(0, 0.5 * rearLoad - lateralTransferR);
  Fz_RR = max(0, 0.5 * rearLoad + lateralTransferR);

  // Rigid body.
  m * der(vx) = Fx + m * yawRate * vy;
  m * der(vy) = FyF + FyR - m * yawRate * vx;
  Izz * der(yawRate) = a * FyF - b * FyR;

  // Roll, as a damped second-order mode about the roll axis.
  der(rollAngle) = rollRate;
  Ixx * der(rollRate) = m * accY * hCg - rollStiffness * rollAngle - rollDamping * rollRate;

  accX = der(vx) - yawRate * vy;
  accY = der(vy) + yawRate * vx;

  // Reaction torque at the handwheel, matching VehicleFMI's sign convention:
  // handwheelTorque is the reaction, and the force-feedback chain negates it to
  // get the torque applied to the driver. Steering left must therefore push the
  // wheel back to the right.
  handwheelTorque = FyF * trail / steerRatio;

  vehicleSpeed = sqrt(vx * vx + vy * vy);
  velX = vx;
  velY = vy;
  yawVel = yawRate;
  roll = rollAngle;
  sideslip = vy / vRef;

  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.001),
    Documentation(info = "<html><p>
Fixture plant for the BobDil FMI path. Presents the
<code>BobLib.Experiments.Standards.VehicleFMI</code> signal boundary with a
simple, smooth, event-free vehicle so that the kernel's Model Exchange loader
and fixed-step integrator can be tested without a MultiBody build.
</p></html>"));
end DilSmokePlant;
