# Portable native visual/gyro grasp regression

The main implementation already contained independent capture-wrist optics,
image-only PnP, shared-gyro own-motion prediction and force-controlled capture,
but a clean checkout lacked the retained initial scene, vision-export runtime
settings and a unified qualification entry. The new opt-in regression resolves
those inputs portably and launches current-checkout backend, UE and simulation
without changing ordinary manual startup or physical/controller parameters.

The runner verifies real native PNG/PnP, complete control phases, gyro cadence,
continuous force support, self-contact, physical fingerprints and independent
free-flight target disturbance. Inputs and acceptance boundaries are described
in `space_sim_server/docs/VISUAL_GYRO_CAPTURE_REPRODUCTION.md`. Raw outputs,
machine paths, authentication databases and compiled artifacts remain outside
version control. Runtime configuration is restored and only owned processes
are closed, including on ordinary failure.
