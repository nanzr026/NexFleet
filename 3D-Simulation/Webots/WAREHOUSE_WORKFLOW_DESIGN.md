# Inspection and integration
The saved world already contains PRODUCT_01 at (-12,-11); generated tests reuse that DEF rather than adding a duplicate. Its separate decorative cartons remain at the original storage location; the transported object is the pallet itself.
Existing Pallet.proto is a visible static Solid with collision geometry. AMRController
already has pickup/delivery states, docking, attach/detach topics and carry/release
checks, but these assume the historical Gazebo joint. WebotsNavigation explicitly
rejects pickup and WebotsPeer disables capability. Neither existing Supervisor
implements attachment. Existing A*, PIBT, LaCAM, physical robot controller and
accepted evidence are frozen by workflow_runs/initial_manifest.json.

Use optional Webots subclasses and a separate environment Supervisor. Never set
AMR poses. Guard pickup by actual distance, orientation, measured speed and ownership.
Lift the physical pallet above the AMR in two visible stages, retain collision
geometry and follow the actual robot pose. Reverse the sequence at delivery.
This is a kinematic follower, not a simulated force-transmitting forklift or load
mass. Existing Pallet geometry is reused. Loaded map clearance is 0.85 m.
Implement P1 before dynamic blockage. No Dashboard changes.

## Confirmed runtime blockages
/warehouse/environment_command accepts JSON BLOCK_AISLE with id,x,y,sx,sy and
UNBLOCK_AISLE with id. The Supervisor rejects insertion within a conservative
clearance around any measured robot. It creates a visible orange Solid with Box
collision geometry and waits for a subsequent physics step before confirmation.
Removal calls the actual scene-node removal before publishing inactive state.

The transient-local /warehouse/environment topic provides a complete versioned
snapshot. Each EnvironmentMap applies confirmed newer versions only and reports
affected occupancy cells. An invalid current A* route stops, replans with the
same planner, publishes updated path intention and reservation metadata.
If the environment changes during an already-agreed multi-agent reservation
session, the adapter fails closed rather than silently invalidating agreement.
Dynamic changes during an active joint session are not yet supported.

Unloaded maps retain .60 m inflation. Mounted-pallet maps use .85 m static
inflation; blockage clearance is 2.0 m while carrying, preserving the existing
controller's conservative 1.85 m loaded LiDAR stop margin. Motor LiDAR thresholds
are unchanged. The environment observer also checks the pallet footprint against
static warehouse rectangles and active blockages because kinematic/static overlap
may not produce a force contact.
