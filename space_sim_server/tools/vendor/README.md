# Official reference example

`scenarioRendezVous.py` is an unmodified copy from AVS Lab Basilisk commit
`43701e745cea88e49615b93ec68558715b2538aa`. This research reference is separately
pinned; the integrated platform currently uses bsk 2.11.1 for its MuJoCo ABI.
Its original ISC copyright and license are retained in the file.

Source: https://github.com/AVSLab/basilisk/blob/43701e745cea88e49615b93ec68558715b2538aa/examples/scenarioRendezVous.py

SHA-256: `1fec9a5659f038118a0542a4a384a99f20dbf139d3b2b23b13665557b38f4a3f`

Run offline with `python tools/run_official_rendezvous.py --output-dir run/official`.
This reference example uses impulsive velocity changes for orbital maneuvers.
The team's LocalMujocoStepper implementation applies bounded forces during integration.
