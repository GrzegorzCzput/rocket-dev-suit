---
status: accepted
---

# Keep preview poses out of rocket definitions

Persist actuator geometry, identity, and mechanical deflection limits in the versioned Rocket Definition, but keep current deflections in a transient Preview Pose. Rocket definitions therefore always describe Neutral Geometry, while the editor and image exporter may render a temporary deflected pose without turning UI state into authoritative geometry.
