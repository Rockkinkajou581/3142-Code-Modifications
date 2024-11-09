
float lemlib::Chassis::crossingexit(Pose current, Pose target, Pose initial, float theta) {
   //take a dot product between the vectors
   Pose offset((current.x - target.x), (current.y - target.y));
   Pose initialoffset ((initial.x - target.x), (initial.y - target.y));
   initialoffset.rotate(-theta/2);
   return ((initialoffset.x * offset.x) + (initialoffset.y * offset.y));
}

