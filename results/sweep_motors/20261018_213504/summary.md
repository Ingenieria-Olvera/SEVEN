# Motor sweep

Max robot mass (kg) that passes. Every pass also requires 2-leg stand + crouch (60°) knee and 15° lean hip-roll holds ≤ τ_cont. 'none' = fails at the lightest mass tried (9.2–11.2 kg). Masses tried go up to 23.8 kg; a value at the top of a combo's range means the limit is higher than tested.

| motor | ratio | act kg | τ_peak | τ_cont | ω0 | max kg @ jump ≥20 cm | max kg @ jump ≥30 cm | max kg @ jump ≥40 cm | max kg holds only | max kg 1-leg 30° | max kg 1-leg 45° |
|---|---|---|---|---|---|---|---|---|---|---|---|
| m8108_100 | 10 | 0.6 | 27.7 | 9.2 | 23.2 | 12.6 | 12.6 | none | 12.6 | 18.3 | 15.5 |
| m8108_100 | 11 | 0.6 | 30.1 | 10.0 | 21.1 | 13.8 | 12.6 | none | 13.8 | 19.5 | 16.1 |
| m8108_100 | 12 | 0.6 | 32.9 | 11.0 | 19.4 | 14.9 | 12.6 | none | 14.9 | 19.5 | 17.2 |
| m8108_100 | 15 | 0.6 | 40.1 | 13.4 | 15.5 | 18.3 | 9.7 | none | 18.3 | 19.5 | 19.5 |
| m6215_170 | 10 | 0.8 | 17.5 | 5.9 | 39.5 | none | none | none | none | 12.4 | 9.9 |
| m6215_170 | 11 | 0.8 | 19.0 | 6.4 | 35.9 | none | none | none | none | 13.6 | 10.5 |
| m6215_170 | 12 | 0.8 | 20.7 | 7.0 | 32.9 | none | none | none | none | 14.2 | 11.2 |
| m6215_170 | 15 | 0.8 | 25.3 | 8.5 | 26.3 | 11.2 | 11.2 | none | 11.2 | 17.3 | 14.2 |
| ea95_100 | 10 | 1.0 | 42.0 | 14.4 | 23.2 | 18.2 | 18.2 | none | 18.2 | 23.8 | 21.0 |
| ea95_100 | 11 | 1.0 | 45.6 | 15.7 | 21.1 | 19.6 | 19.6 | none | 19.6 | 23.8 | 22.4 |
| ea95_100 | 12 | 1.0 | 49.8 | 17.1 | 19.4 | 21.0 | 19.6 | none | 21.0 | 23.8 | 23.8 |
| ea95_100 | 15 | 1.0 | 60.7 | 20.9 | 15.5 | 23.8 | 16.1 | none | 23.8 | 23.8 | 23.8 |
| m10_100 | 10 | 0.9 | 50.4 | 16.0 | 23.2 | 20.1 | 20.1 | none | 20.1 | 22.8 | 22.8 |
| m10_100 | 11 | 0.9 | 54.8 | 17.4 | 21.1 | 21.5 | 21.5 | none | 21.5 | 22.8 | 22.8 |
| m10_100 | 12 | 0.9 | 59.8 | 19.0 | 19.4 | 22.8 | 22.8 | none | 22.8 | 22.8 | 22.8 |
| m10_100 | 15 | 0.9 | 72.9 | 23.2 | 15.5 | 22.8 | 21.5 | none | 22.8 | 22.8 | 22.8 |

Wheel (5010 + 6:1, hand calc, not simulated yet: needs e3 balance):
| robot kg | max base accel m/s² | max lean it can catch |
|---|---|---|
| 10 | 7.3 | 37° |
| 12 | 6.1 | 32° |
| 14 | 5.2 | 28° |
| 16 | 4.6 | 25° |
| 18 | 4.1 | 22° |
