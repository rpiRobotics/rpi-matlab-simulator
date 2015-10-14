% Returns the quaternion inverse of quaternion q.
function qinv = qtinv( q )
  qinv  = qtconj( q )./(sum(q.^2,1)*ones(4,1));
end

