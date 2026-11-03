# Personal cloud workspace integration

The AOC branches are integrated with server main e27c374 and adapter main dd23420 in a personal workspace. Capture lifecycle and rendezvous initialization are both retained when resolving the server merge.

Rendezvous flights previously sent render frames to port 5558 even when the platform renderer used a custom port. The runtime now propagates the configured render port to team flights and historical playback. Platform/backend entry scripts also expose the rendezvous state port and check it before startup. Existing default ports remain compatible.

Cloud validation uses separate ports 28000/28766/28767/28768/25558/28888/28080. Source, runtime, accounts and outputs are contained in the personal workspace; other workspaces and services are not stopped.

Validation evidence is written under the workspace validation directory: full seven-task acceptance, server/adapter CI, frontend/signalling tests, native UE build/automation, and browser integration. Only actual successful results in these receipts constitute completion.

The stock UE signalling server also opened SFU port 8889 regardless of the custom streamer/player ports. PixelSfuPort is now propagated explicitly; this workspace uses 28889.

Python 3.12 waits for accepted sockets in Server.wait_closed(). SimulationHub now closes its active peer before waiting; RendezvousHub closes all accepted writers. Regression tests exercise shutdown with live/idle connections, and the reconnect test releases its peer before awaiting listener shutdown.

Official example ephemerides are pinned in the workspace vendor/supportData directory via BSK_SUPPORT_DATA_CACHE. UE asset preparation regenerates machine-local uasset import metadata; these runtime asset differences are indexed separately from source commits.


The rendezvous page now gets its player port and streamer ID from /api/client-config, preventing a personal instance from accidentally displaying another workspace. Frontend tests cover custom/default ports, IPv6, streamer-name escaping, and invalid configurations.
