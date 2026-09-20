# RoboMaster rulebook baseline

This project currently uses two local source documents. They remain local raw
references and are not redistributed by the repository:

| Document | Status | SHA-256 |
|---|---|---|
| `references/rules/UTF-8__RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V2.2.0（20260807）.pdf` | RMUC 2026 V2.2.0 rules, 159 pages | `88ae3c7d0bbd7312c8095eff2603910437e292a0b53f46c9378f26144c09a6f5` |
| `references/rules/RoboMaster 2027 高校系列赛规则变更前瞻手册（20260909）.pdf` | RM2027 preliminary changes, 19 pages; formal rules will supersede it | `1f8fb31f9983bd6854870d038fabe0c67ff7102b7bddf0425dec872948ab1b19` |

## Guidance lamp and target

The RMUC 2026 rules, pages 40–41, identify the dart detection module as a
small armor module, dart detection sensor and dart guidance lamp. The lamp emits
green visible light around 520 nm, has approximately 10 cd luminous intensity
when treated as a point source, and has an approximately 55 mm luminous diameter.
It was intended to guide darts. These values now set the provisional marker
diameter and metadata in `configs/base_target.json`; MuJoCo currently renders a
colored disk and does not simulate its photometry or camera response.

Last-season match logic did not keep the base lamp continuously lit: page 100
states that the outpost lamp is on and base lamp off while the outpost survives,
then swaps them after the outpost is destroyed. Page 116 also permits the dart
detection module and lamp to move together by ±280 mm along the field-width
direction for some dart target modes. These are 2026 dart rules, not confirmed
RM2027 aerial-robot target behavior. Vision design must tolerate an unavailable
marker until the RM2027 formal rules establish its state and motion.

The photographed/current-field layout used by this project places the green
marker above the intended blue contact patch. The 2026 dart-module drawing shows
a different relative arrangement. The older drawing supplies lamp optical and
size evidence, but it does not override the user's RM2027 target identification
or establish the current target pose.

## Impact-speed evidence and open 2027 rule

The RMUC 2026 rules, PDF page 68 (printed page 67), list the armor module's
effective detection-speed thresholds as greater than 12 m/s for a 17 mm
projectile and greater than 10 m/s
for a 42 mm projectile at large, small, base and outpost armor modules. The same
page specifies the projectile velocity component normal to the armor's struck
surface as the detection value; it is not merely total airspeed. The section
also says a physical collision may sometimes be classified as projectile damage,
while the 2026 competition rules prohibit deliberately causing damage by
collision. The 10 m/s figure is therefore evidence about the existing armor
detection system, not an authorized 2026 aircraft-impact rule.

The RM2027 preview changes the intended role: an aerial robot may directly
strike base or outpost armor and cause projectile damage. It does not yet define
whether the aircraft is classified as a 42 mm projectile, the minimum impact
speed, the damage value, the valid contact area, or the referee-system matching
logic. The working engineering hypothesis is that an impact above 10 m/s may be
needed for the armor to recognize large-projectile damage. This is a provisional
research target only; the formal RM2027 rule must define the acceptance gate.

High-speed impact and recovery are consequently one coupled mission problem.
The aircraft must produce a valid armor event, survive the impulse without an
unsafe propeller or guard contact, arrest post-impact translation and rotation,
recover controlled flight, and return to its assigned dock slot. A geometric
touch or a simulated contact event alone cannot establish this capability. Each
test must report the pre-impact velocity vector and its armor-normal component.

## RM2027 aerial robot and dock

The September 9 preview, pages 10–12, describes an aerial robot that may directly
strike outpost/base armor, actively intercept darts and projectiles, or collect
information before returning to the dock. It requires at least ten seconds of
sustained hover, automatic operation, flexible propellers, an enclosing guard
that keeps propellers and aircraft from contacting armor during a strike, and an
emergency-stop receiver with a safety operator. It does not allow a projectile
launcher. Multi-vehicle communication is allowed.

The preliminary mass-size relation is
`y = -0.001225 x + 0.3225`, with mass `y` in kg, maximum dimension `x` in mm,
and `y` between 0.15 and 0.249 kg. Therefore:

- 180 g permits about 116.3 mm maximum length, width and height.
- 120 mm permits about 175.5 g maximum mass.

The current 180 g, 120 mm simulation proxy does not satisfy both preliminary
values simultaneously. It remains a development envelope until actual mass and
dimensions are measured and the formal rule is published.

The preview permits one aerial-robot/dock system, up to four aircraft in the
dock, a 300 × 300 × 200 mm maximum stored dock size, 300 Wh supply capacity and
30 V maximum working voltage. The dock may provide initial propulsion only
within a 15° cone about its vertical normal. The current CAD audit measures the
dock as 300 × 300 × 200 mm and models four slots, matching those two preliminary
limits; electrical, emergency-stop, full-enclosure and launch-cone compliance
are not yet established.

## Development implications

- Prioritize one aircraft completing repeatable full missions under
  MILESTONES.md before expanding quantity. Retain ten-second hover, armor
  impact, post-impact stabilization, retreat and dock return as objectives.
- Keep low-speed contact as an integration test. Develop high-speed impact with
  a staged speed envelope and measured post-impact recovery, without claiming
  that 10 m/s is the final RM2027 threshold before the formal rule is published.
- Treat interception as a later mission, as already planned.
- Do not use GPS navigation in the intended aircraft stack; M10G-5883 is for its
  compass only. MTF-02P is the confirmed optical-flow/range sensor.
- Keep rule claims versioned. The 2027 document is a preview with multiple TBD
  limits, so future formal rules require a new audit before compliance claims.
