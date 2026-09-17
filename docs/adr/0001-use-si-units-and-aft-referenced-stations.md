---
status: accepted
---

# Use SI units and aft-referenced stations

Store lengths in metres and angles in radians, make those units explicit with `_m` and `_rad` field suffixes, and measure component stations forward from an Aft Datum at the centre of the body tube's rear plane. This makes a zero Fin Station place the fin's root trailing edge at the rear of the rocket, matching how the geometry is authored; calculations derived from nose-referenced sources such as the Barrowman method must transform coordinates at their boundary.
