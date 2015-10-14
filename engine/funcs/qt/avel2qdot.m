% Converts a angular velocity to a time-derivative of a quaternion

function qdot = avel2qdot(q, w)

  % determine G
  G = [-q(2) q(1) -q(4) q(3); -q(3) q(4) q(1) -q(2); -q(4) -q(3) -q(2) q(1)];

  % compute quaternion
  qdot = 0.5*G'*w;

