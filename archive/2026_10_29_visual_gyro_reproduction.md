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

Integration preserves both the rendezvous updates in main@94bdd75 and collection
updates in main@6c53480. Native build calls follow their relocated assets tools;
capture catalogs use the manifest's frozen base. The authored capture XML and all
its body inertials remain identical. The final review's process-liveness, complete
gyro-session coverage and failed-shutdown recovery findings have regression tests.
The existing contact_seek branch is accepted as closure only when actual inward
jaw commands are recorded, followed by the required physical support.

The first completed nominal baseline passed all11 physical/image/gyro gates;
its full numbers and latest integration checks are documented in the reproduction
guide. Prior setup failure, deliberate interruption and scorer-branch rejection
are retained outside Git rather than overwritten. The final integration's native
result and protected-main publication status are tracked in PR38.
